import math
import random
from typing import List

import rospy

from numpy import argmax, delete, max

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseActionFeedback

from bayesian_classifier import ObjectLocator, Location
from bayesian_object_finder.srv import GetParentClass, GetParentClassResponse, GetObjectLocation
from bayesian_object_finder.srv import GetObjectLocationResponse, GetObjectLocationRequest
from bayesian_object_finder.srv import GetSceneObjectList, GetSceneObjectListResponse


class ObjectLocatorNode(ObjectLocator):
    """
    ROS wrapper class for the GPC class. Includes all services/topics needed for the system to work.
    """

    # Map from Location names to actual poses. Manually defined to be in the "center" of the region.
    LOCATION_TO_POSE = {
        Location.CupboardTop.name: Pose(Point(7.695517539978027, -4.043793201446533, 0.0),
                                     Quaternion(0, 0, -0.2785554066867665, 0.960420160870007)),
        Location.Cupboard.name: Pose(Point(7.695517539978027, -4.043793201446533, 0.0),
                                     Quaternion(0, 0, -0.2785554066867665, 0.960420160870007)),
        Location.Refrigerator.name: Pose(Point(8.724411010742188, -1.0257198810577393, 0.0),
                                         Quaternion(0, 0, -0.9991713391427528, 0.040701781726087134)),
        Location.Table.name: Pose(Point(6.546411514282227, 0.9594424962997437, 0.0),
                                  Quaternion(0.0, 0.0, 0.03325770580824047, 0.9994468094923173))
    }

    # List of location names from enum (used for logging/HMI purposes)
    LOCATIONS = [loc.name for loc in Location]

    # Radius of circle around central pose in which random poses can be defined for navigation.
    SEARCH_RADIUS = 1  # [m]
    # Total time spent in each search attempt
    SEARCHING_TIME = 15  # [s]
    # Total number of attempts that can be made at each location
    MAXIMUM_NUM_OF_SEARCHES = 5

    #### Flags used in navigation feedback ####

    NAVIGATION_FAILED = 4

    # Tolerance used when comparing current pose and goal pose
    NAVIGATION_GOAL_REACHED_TOL = 1e-1


    #### Counters and helper variables ####
    num_of_searches = 0
    navigation_finished = False
    navigation_failed = False
    searching = False
    searching_start_time = None
    nav_goal_published = False
    object_name = ""

    probabilities = []

    center_pose = Pose()
    target_pose = PoseStamped()
    current_pose = PoseStamped()

    def __init__(self, train = False, N = 500):
        # Initialise locator model
        super().__init__(train, N)

        # Service that returns object location with probability.
        # Also begins navigation procedure towards said object.
        rospy.Service('get_obj_location', GetObjectLocation, self.handle_get_object_location)

        # Publisher used to publish the pose that robot will attempt to navigate to
        self.nav_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=4)

        # Receives current status of navigation as well as current robot pose.
        self.nav_status_subscriber = rospy.Subscriber("/move_base/feedback",
                                                      MoveBaseActionFeedback,
                                                      self.move_base_feedback_callback)

        # Used to determine that classifier model is loaded/trained.
        rospy.loginfo("Ready to get object location.")

    def move_base_feedback_callback(self, msg: MoveBaseActionFeedback):
        """
        Uses information from navigation stack to update current navigation status as well as
        current known robot pose.

        Args:
            msg (MoveBaseActionFeedback): message received from /move_base/feedback topic.
        """
        # Flag that indicates navigation status.
        status_flag: int =  msg.status.status

        # Pose from topic
        self.current_pose = msg.feedback.base_position

        # distance to goal pose
        dx: float = self.current_pose.pose.position.x - self.target_pose.pose.position.x
        dy: float = self.current_pose.pose.position.y - self.target_pose.pose.position.y
        dz: float = self.current_pose.pose.position.z - self.target_pose.pose.position.z

        d: float = (dx**2 + dy**2 + dz**2)**0.5

        # Only way to determine if navigation is finished. Status flag does not correctly indicate
        # if navigation is successfully completed (i.e. never becomes 3)
        self.navigation_finished = d <= ObjectLocatorNode.NAVIGATION_GOAL_REACHED_TOL

        # In case navigation fails due to lack of a plan or any other reason. Typically
        # timeout occurs before this.
        self.navigation_failed = status_flag == ObjectLocatorNode.NAVIGATION_FAILED


    def get_parent_client(self, obj_name: str) -> str:
        """
        Client of get_parent service. Returns name of parent class of a class from the ontology.
        In case there are multiple parent classes, first class in the list is returned. This filter
        step occurs within the server rather than the client.

        Args:
            obj_name (str): name of current object/class

        Returns:
            str: name of parent class
        """
        rospy.wait_for_service('get_parent')
        try:
            get_parent = rospy.ServiceProxy('get_parent', GetParentClass)
            resp1: GetParentClassResponse = get_parent(obj_name)
            return resp1.parent
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def get_scene_object_list_client(self, obj_name: str) -> List[str]:
        """
        Client of get_scene_object_list service. Checks if obj_name has been seen before.
        Returns a list of all seen objects when "all" is the obj_name.
        Args:
            obj_name (str): name of object we wish to know if it has been seen

        Returns:
            List[str]: names/name of seen object.
        """
        rospy.wait_for_service('get_scene_object_list')
        try:
            get_scene_object_list = rospy.ServiceProxy('get_scene_object_list', GetSceneObjectList)
            resp1: GetSceneObjectListResponse = get_scene_object_list(obj_name)
            return resp1.objects
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def handle_get_object_location(self, req: GetObjectLocationRequest) -> GetObjectLocationResponse:
        """
        Server for service get_object_location. When provided with an object name, uses GPC to find
        list of most likely locations. Set navigation strategy to use the most likly location and
        begin the process of searching for the object at that area.

        Args:
            req (GetObjectLocationRequest): request from server. Includes object name.

        Returns:
            GetObjectLocationResponse: response from server. Includes if object is found,
            name of location, estimated probability of the object being there,
            and central pose of that region.
        """
        self.object_name = req.obj_name
        rospy.loginfo("Finding most likely location for %s."%self.object_name)

        # Use get_parent service to get parent class
        obj_parent = self.get_parent_client(self.object_name)

        # Remove ontology path from name
        parent_name = obj_parent.split('#')[-1]

        name_for_locator = None

        # if RequestableObject is parent, use the current object name, otherwise, the
        # parent class name should work.
        if parent_name == "RequestableObject":
            name_for_locator = self.object_name
        else:
            name_for_locator = parent_name

        # Use GPC for location + probability estimates
        self.probabilities = self(name_for_locator)

        self.probabilities = self.probabilities[0]

        best_estimate_prob = argmax(self.probabilities)

        best_location_estimate = self.LOCATIONS[best_estimate_prob]

        location_pose = ObjectLocatorNode.LOCATION_TO_POSE.get(best_location_estimate, None)

        # keep rounded version of probability for logging/HMI purposes.
        prob_pretty = round(float(self.probabilities[best_estimate_prob]), 3) * 100

        # If the most likely location has a central pose. If not, something has gone wrong
        if location_pose:
            msg ="discovered object %s at %s with %s certainty."%(
                self.object_name, best_location_estimate, prob_pretty)

            rospy.loginfo(msg)

            self.searching = True
            self.searching_start_time = rospy.Time.now()
            self.center_pose = location_pose
            self.num_of_searches = 0
            self.search_for_object(self.searching_start_time, rospy.Time.now())

        return GetObjectLocationResponse(True,
                                         best_location_estimate,
                                         float(self.probabilities[best_estimate_prob]),
                                         location_pose)

    def get_and_publish_random_pose(self):
        """
        Helper function that generates a random pose around a central pose, then publishes it using
        the navigation publisher.
        """
        # get random pose
        random_search_pose = self._generate_random_pose(self.center_pose)

        # Generate stamped pose message
        header = Header(1, rospy.Time.now(), "map")
        random_search_pose_stamped = PoseStamped(header, random_search_pose)
        self.target_pose = random_search_pose_stamped

        # publish given stamped pose
        self.nav_publisher.publish(random_search_pose_stamped)

        # logging/setting flags
        msg ="Navigating to:" + str(random_search_pose_stamped.pose)
        rospy.loginfo(msg)
        self.nav_goal_published = True

    def check_num_attempts(self):
        """
        Helper function that checks if the maximum number of searches has been reached.
        If so, get the next most likely location. If the classifier is not confident enough, then
        the robot did not find the object at all.
        """
        # Max search attempts at a location is reached, attempt a new one.
        if self.num_of_searches >= ObjectLocatorNode.MAXIMUM_NUM_OF_SEARCHES:

            # Stop the navigation by setting current pose as the next goal.
            self.nav_publisher.publish(self.current_pose)
            rospy.logerr("Could not find %s within alloted time. Changing search location",
                         self.object_name)

            # Check if there are more locations available
            if len(self.probabilities) > 1:

                # Remove the most likely location (i.e. where the robot has been searching so far.)
                self.probabilities = delete(self.probabilities, argmax(self.probabilities))

                # If other locations liklihoods are negligible, ignore them and stop the search.
                if max(self.probabilities) < 0.08:
                    rospy.logerr("Could not find %s at any of known locations.", self.object_name)
                    # Reset necessary helper variables/flags.
                    self.probabilities = []
                    self.searching = False
                    self.nav_goal_published = False

                # Next most likely location found.
                else:
                    best_location_prob = argmax(self.probabilities)
                    best_location_estimate = self.LOCATIONS[best_location_prob]
                    location_pose = ObjectLocatorNode.LOCATION_TO_POSE.get(best_location_estimate,
                                                                           None)
                    # If the most likely location has a central pose.
                    # If not, something has gone wrong.
                    if location_pose:
                        msg ="Searching at %s with %s certainty."%(
                            best_location_estimate,
                            round(float(self.probabilities[best_location_prob]), 3) * 100)
                        rospy.loginfo(msg)

                        self.searching = True
                        self.searching_start_time = rospy.Time.now()
                        self.center_pose = location_pose
                        self.num_of_searches = 0
                        self.search_for_object(self.searching_start_time, rospy.Time.now())

            # Could not find object at any of known locations. Stops the search.
            else:
                rospy.logerr("Could not find %s at any of known locations.", self.object_name)
                self.searching = False
                self.nav_goal_published = False

    def search_for_object(self, searching_start_time: rospy.Time, current_time: rospy.Time) -> int:
        """
        function that contains the logic for searching.

        Args:
            searching_start_time (rospy.Time): Time at which this search attempt started.
            current_time (rospy.Time): Current time.

        Returns:
            int: Search status flag. 0 indicates search attempt is ongoing,
            1 indicates no search/search attempt failed.
        """
        # "default" case.
        if not self.searching:
            return 1

        self.check_num_attempts()

        time_passed = (current_time - searching_start_time).to_sec()

        # helpful to know when the attempt will be reset.
        rospy.logdebug("Time Passed since search: "+str(time_passed))

        # Too much time passed when searching
        if time_passed >= ObjectLocatorNode.SEARCHING_TIME:
            self.num_of_searches += 1
            # As long as max attempts are not reached, try one more time
            if self.num_of_searches < ObjectLocatorNode.MAXIMUM_NUM_OF_SEARCHES:
                rospy.logwarn("Trying new location. Attemp nr %s", self.num_of_searches)
                self.get_and_publish_random_pose()
            return 1

        # Happens either at search start or at navigation fail
        if not self.nav_goal_published or self.navigation_failed:
            # If navigation fail, increment search counter and log.
            if self.navigation_failed:
                self.num_of_searches += 1
                rospy.loginfo("Navigation failed. Trying new location. Attemp nr %s",
                              self.num_of_searches)

            self.get_and_publish_random_pose()
            return 0

        #### logic to see if desired object has been seen ####

        list_of_seen_objects: List[str] = self.get_scene_object_list_client("all")

        # since the model name may be different from the provided object name,
        # pre-processing on the strings must be done before comparisons.
        list_of_seen_objects = [a.lower() for a in list_of_seen_objects]

        object_is_seen = [self.object_name.lower() in seen_object for
                          seen_object in list_of_seen_objects]

        dx = self.current_pose.pose.position.x - self.target_pose.pose.position.x
        dy = self.current_pose.pose.position.y - self.target_pose.pose.position.y
        dz = self.current_pose.pose.position.z - self.target_pose.pose.position.z

        d: float = (dx**2 + dy**2 + dz**2)**0.5

        self.navigation_finished = d <= ObjectLocatorNode.NAVIGATION_GOAL_REACHED_TOL

        if any(object_is_seen):
            # Object found, stop navigation.
            self.nav_publisher.publish(self.current_pose)
            rospy.loginfo("Object has been found!")
            # reset flags
            self.searching = False
            self.nav_goal_published = False
        elif self.navigation_finished:
            # navigation goal reached but object not seen. increment counter, log, and reset.
            self.num_of_searches += 1
            rospy.loginfo("Navigation complete but object not found."\
                          "Publishing new navigation goal.")
            self.searching_start_time = rospy.Time.now()
            self.get_and_publish_random_pose()

        return 0

    def _generate_random_pose(self, center_pose: Pose, radius: float = SEARCH_RADIUS) -> Pose:
        """
        Generate a random pose around the provided central pose.

        Args:
            center_pose (Pose): Pose at center of the circle.
            radius (float, optional): radius of random circle. Defaults to SEARCH_RADIUS.

        Returns:
            Pose: new random pose.
        """
        random_angle = random.uniform(0, 2 * math.pi)  # Generate a random angle
        position = Point(
            center_pose.position.x + radius * math.cos(random_angle),
            center_pose.position.y + radius * math.sin(random_angle),
            0.0)
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(random_angle / 2.0)
        quat.w = math.cos(random_angle / 2.0)
        random_pose = Pose(position, quat)
        return random_pose


#### code of the node ####

rospy.init_node("object_locator")

obj_locator = ObjectLocatorNode()

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    status = obj_locator.search_for_object(obj_locator.searching_start_time, rospy.Time.now())

    # If not searching or search failed, reset search clock
    if status > 0:
        obj_locator.searching_start_time = rospy.Time.now()

    rate.sleep()

rospy.spin()

