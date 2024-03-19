#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <bayesian_object_finder/UpdateObjectList.h>
#include <bayesian_object_finder/SetInitTiagoPose.h>
#include <bayesian_object_finder/GetSceneObjectList.h>

#include <cctype> //library needed for the toupper function

class WorldInfo
{
private:

    std::string subs_topic_name_;        ///< gazebo model state topic name
    std::string srv_update_obj_name_;        ///< name of the service provided by the map generator node
    ros::Subscriber sub_gazebo_data_;     ///< Subscriber gazebo model_states

    std::string srv_assert_knowledge_name_; ///< Name of the service to assert knowledge in the ontology

    std::string srv_get_scene_name_;       ///< service name to send a target object in the scene

    std::vector<std::string> v_seen_obj_;  ///< List of objects seen by the robot and sent to the map generator node

    ros::ServiceServer get_scene_obj_srv_; // Advertise service to send the pose of a target object in the scene

    ros::ServiceClient client_map_generator_; ///< Client to request the object list update in the map generator node

    ros::ServiceClient client_reasoning_; ///< Client to assert objects in the knowledge base


public:

    WorldInfo(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created world info");

        // This objects will not be sent to the Map generator node
        v_seen_obj_.push_back("tiago");
        v_seen_obj_.push_back("ground_plane");

        subs_topic_name_="/gazebo/model_states";

        // create client and wait until service is advertised
        srv_update_obj_name_="update_object_list";
        client_map_generator_ = nh.serviceClient<bayesian_object_finder::UpdateObjectList>(srv_update_obj_name_);

        // create a client for the reasoning node service to assert knowledge
        srv_assert_knowledge_name_ = "assert_knowledge";
        client_reasoning_ = nh.serviceClient<bayesian_object_finder::UpdateObjectList>(srv_assert_knowledge_name_);

        srv_get_scene_name_ = "get_scene_object_list";
        get_scene_obj_srv_ = nh.advertiseService(srv_get_scene_name_, &WorldInfo::srv_get_scene_obj_callback, this);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_update_obj_name_.c_str());
        bool service_found = ros::service::waitForService(srv_update_obj_name_, ros::Duration(30.0)); // You can adjust the timeout as needed

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_update_obj_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: "<<srv_update_obj_name_);

        // Create subscriber to receive the commanded turtle state. This state will be generated from a trajectory generator
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &WorldInfo::topic_callback, this);
    };

    ~WorldInfo()
    {

    };

private:

    /**
    * @brief Function to always have the first letter of a string as capital letter
    *
    */

    std::string capitalizeFirstLetter(const std::string& s)
    {
        std::string capitalized_s = s;

        // first verify if the string is not empty
        if(!capitalized_s.empty())
        {
            //convert the first character to uppercase
            capitalized_s[0] = std::toupper(capitalized_s[0]);
        }

        return capitalized_s;

    }

    /**
     * @brief Callback function to receive the Gazebo Model State topic
     *
     * @param msg message with the current Gazebo model state
     */
    void topic_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {

        //Get robot Pose
        geometry_msgs::Pose tiago_pose;
        // Search for tiago pose
        auto it = std::find( msg->name.begin(),  msg->name.end(), "tiago");
        if (it != msg->name.end())
        {
            // Calculate the index
            int index = std::distance(msg->name.begin(), it);
            tiago_pose=msg->pose.at(index);
        }

        // search new objects in the scene
        for (int i = 0; i < msg->name.size(); i++)
        {
            // Get obj position
            // Get tiago position
            // Compare distances, if within range then check the v_seen_list
            // if the obj is not in the list add it and send it to the srv

            // Get object pose
            //Identify the right variable that contains the correct object pose (0.5pts)
            geometry_msgs::Pose obj_pose= msg->pose[i];

            // get distance from tiago to obj[i]
            // obtain the dx distance between the robot and the objects (0.5 pts)
            double dx = tiago_pose.position.x - obj_pose.position.x;
            // obtain the dy distance between the robot and the objects (0.5 pts)
            double dy = tiago_pose.position.y - obj_pose.position.y;
            // compute the distance between the tiago and the objects (1pt)
            double d = sqrt(dx*dx+dy*dy);

            //IF the robot is closer to the seen objects, then request the service
            if (d<1.1)
            {
                //Identify the object seen (0.5 pts)
                std::string s= msg->name[i];
                // Search for the obj name in the seen_list
                auto it = std::find(v_seen_obj_.begin(), v_seen_obj_.end(), s);

                // If the obj name is not found in the seen vector, this means that the robot has seen a new object for the first time and it should add it to the seen vector and call the service update_object_list
                if (it == v_seen_obj_.end()) {

                    bayesian_object_finder::UpdateObjectList srv;

                    // send the new seen object to the service (0.25 pts)
                    srv.request.object_name= s;
                    // send the pose of the seen object to the service (0.25 pts)
                    srv.request.object_pose= obj_pose;

                    if (client_map_generator_.call(srv))
                    {
                        ROS_INFO_STREAM("Object List Updated?: "<< (int)srv.response.confirmation);

                        if(srv.response.confirmation)
                        {
                            v_seen_obj_.push_back(s);

                            ROS_INFO_STREAM("Object ["<<s<<"] added to the list");
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Failed to call service "<<srv_update_obj_name_);
                    }

                    // call the reasoning service
                    bayesian_object_finder::UpdateObjectList srv_reasoning;
                    //before calling the srv, make sure that the first letter of the "s" is capitalized
                    std::string capitalized_s = capitalizeFirstLetter(s);
                    srv_reasoning.request.object_name= capitalized_s; //I just need to send the ID of the object

                    if(client_reasoning_.call(srv_reasoning))
                    {
                        ROS_WARN_STREAM ("Creating a new instance of: "<< capitalized_s);

                        ROS_INFO_STREAM ("New instance included in knowledge: "<<(int)srv_reasoning.response.confirmation);

                        if(srv_reasoning.response.confirmation)
                        {
                            v_seen_obj_.push_back(s);
                            ROS_INFO_STREAM("Object ["<<s<<"] added to the list");
                        }

                    }
                    else
                    {
                        ROS_ERROR_STREAM("Failed to call service "<<srv_assert_knowledge_name_);
                    }
                }
            } //if d
        }//for msg size

        //If you want to print the objects that the robot has seen so far, just uncomment the for
        //for (size_t i = 2; i < v_seen_obj_.size(); i++)
        // {
        //     ROS_INFO_STREAM("["<<i<<"]: "<<v_seen_obj_.at(i));
        // }

    } // callback


    /**
     * @brief Callback function for the service that sends the info of a target object in the scene. This service could be use, for example, by the controller node to find the target position
     *
     * @param Request name of the requested object. Use "all" to get the list of all the available objects.
     * @param Respose response is a list of objects with name and pose.
     */
    bool srv_get_scene_obj_callback(bayesian_object_finder::GetSceneObjectList::Request &req,
                                    bayesian_object_finder::GetSceneObjectList::Response &res)
    {
        // ROS_INFO_STREAM("Target object: " << req.object_name);

        // If the requested object name is "all", we should send all the available objects

        if (req.object_name == "all")
        {
            // copy all the pairs (names,poses) into the object list (0.5 pts)
            for (auto &&name : v_seen_obj_)
            {
                res.objects.push_back(name);
            }

            res.obj_found = true;
            return res.obj_found;
        }

        int target_id = -1;

        for(int i=0;i < v_seen_obj_.size();i++)
        {
            if(v_seen_obj_.at(i) == req.object_name)
            {
                target_id = i;
                break;
            }
        }

        if (target_id != -1)
        {
            // Push the information that is obtained about the name and pose of the object found in the list in the response variable "res" (0.5pts)
            res.objects.push_back(v_seen_obj_.at(target_id));
            res.obj_found = true;
        }
        else
        {
            // target object is not in the list
            res.obj_found = false;

            std::stringstream s;

            s<<"The target object [" << req.object_name << "] is not in the list. \nAvailable Objects";

            for (auto &&name : v_seen_obj_)
            {
                s<<name<<"\n";
            }

            res.message = s.str();

            ROS_ERROR_STREAM("The target object [" << req.object_name << "] is not in the list");
        }

        return res.obj_found;
    }

}; // Class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "percept_node");
    ros::NodeHandle nh;

    WorldInfo myPercept(nh);

    ros::spin();

    return 0;
}
