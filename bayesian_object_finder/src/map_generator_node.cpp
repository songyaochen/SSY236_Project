#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <bayesian_object_finder/UpdateObjectList.h>
#include <bayesian_object_finder/SetInitTiagoPose.h>

#include <gazebo_msgs/ModelStates.h>

class MapGenerator
{
    private:


        std::string srv_update_obj_name_;        ///< service name for new obj
        ros::ServiceServer update_obj_list_srv_; // Advertise service to update obj list
        std::vector<std::pair<std::string, geometry_msgs::Pose>> map_objs_; ///< List of objects in the scene

        ros::Timer tf_timer_;                 ///< Timer to run a parallel process
        tf::TransformBroadcaster broadcaster_; ///< TF broadcaster variable

    public:

        MapGenerator(ros::NodeHandle& nh)
        {
            ROS_WARN_STREAM("Created world map");

            srv_update_obj_name_="update_object_list";

            update_obj_list_srv_ = nh.advertiseService(srv_update_obj_name_, &MapGenerator::srv_update_callback, this);


            // publish the current position
            tf_timer_ = nh.createTimer(ros::Duration(1), &MapGenerator::tf_timer_callback, this);


        };

        ~MapGenerator()
        {

        };

    private:

    /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the list
     * @param Respose response from the service when the object has been added (true/false)
    */
    bool srv_update_callback(bayesian_object_finder::UpdateObjectList::Request  &req,
            bayesian_object_finder::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: "<<req.object_name);
        ROS_INFO_STREAM("Object Pose: "<<req.object_pose);

        //Push the information that is obtained from the client via the request variables
        // Use the correct variables here (0.5 pt)
        map_objs_.push_back(std::make_pair(req.object_name, req.object_pose));

        for (size_t i = 0; i < map_objs_.size(); i++)
        {
            ROS_INFO_STREAM(map_objs_.at(i).first<<": "<<map_objs_.at(i).second);
        }

        res.confirmation = true;
        return res.confirmation;
    }

    void tf_timer_callback(const ros::TimerEvent& e)
    {
        std::vector<geometry_msgs::TransformStamped> v_ts;

        // Get the current time
        ros::Time aux_time = ros::Time::now();

        for (size_t i = 0; i < map_objs_.size(); i++)
        {
            geometry_msgs::TransformStamped ts;

            std::string object_name = map_objs_.at(i).first; // use the correct variable (0.25 pts)
            geometry_msgs::Pose obj_pose = map_objs_.at(i).second; // use the correct variable (0.25 pt)

            // TF object to populate our TF message
            tf2::Transform tf;

            // use the correct variable to define the right object position (0.5 pts)
            tf.setOrigin(tf2::Vector3(obj_pose.position.x, obj_pose.position.y, obj_pose.position.z));

            // use the correct variable to define the right object orientation (0.5 pts)
            tf.setRotation(tf2::Quaternion(obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w));

            // Transform the TF object to TF message
            ts.transform = tf2::toMsg(tf);

            // Set the reference frame for the TF (parent link)
            // Define the right reference frame (0.5 pts)
            ts.header.frame_id = "world";
            // Set the time stamp for the message
            ts.header.stamp = aux_time;
            // Set the name for the TF
            ts.child_frame_id = object_name;

            //// To visualize objects in Rviz we need to publish its corresponding TF
            // Create TF msg
            v_ts.push_back(ts);
        }

        //Once all the information of the TFs is obtained, then we broadcast this data to Rviz
        broadcaster_.sendTransform(v_ts);

    }

};//class MapGenerator

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_generator_node");
    ros::NodeHandle nh;

    MapGenerator myMapGenerator(nh);

    ros::spin();

    return 0;
}
