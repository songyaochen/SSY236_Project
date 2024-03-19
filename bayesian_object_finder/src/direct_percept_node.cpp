#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>

class DirectWorldInfo
{
private:

    std::string subs_topic_name_;        ///< gazebo model state topic name
    ros::Subscriber sub_gazebo_data_;     ///< Subscriber gazebo model_states

public:

    DirectWorldInfo(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created world info");

        subs_topic_name_="/gazebo/model_states";

        // Create subscriber to receive the commanded turtle state. This state will be generated from a trajectory generator
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &DirectWorldInfo::topic_callback, this);
    };

    ~DirectWorldInfo()
    {

    };

private:

/**
   * @brief Callback function to receive the Gazebo Model State topic
   *
   * @param msg message with the current Gazebo model state
   */
  void topic_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
     static tf::TransformBroadcaster broadcaster;
    // Create a TF vector that stores the pose of the publish multiple TFs
    std::vector<geometry_msgs::TransformStamped> v_ts;

    // Get the current time
    ros::Time aux_time = ros::Time::now();

    // search new objects in the scene 
    for (int i = 0; i < msg->name.size(); i++)
    {
        ///////////// TF broadcast

        geometry_msgs::TransformStamped ts;
        

        std::string model_name = msg->name[i];
        geometry_msgs::Pose pose = msg->pose[i];

        // TF object to populate our TF message
        tf2::Transform tf;

        // set the correct origin and rotation of the variable tf. You need to modify the next two lines, the current values are incorrect, fix them with the correct values!
        tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
        tf.setRotation(tf2::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));

        // Transform the TF object to TF message
        ts.transform = tf2::toMsg(tf);

        // Set the reference frame for the TF (parent link)
        //TODO: You need to also make some modification to the launch file. 
        // You need to check which frame_id is defined in the launch file as the reference frame 
        ts.header.frame_id = "world";
        // Set the time stamp for the message
        ts.header.stamp = aux_time;
        // Set the name for the TF
        ts.child_frame_id = model_name + "_direct";

         //// To visualize objects in Rviz we need to publish its corresponding TF
        // Create TF msg
        v_ts.push_back(ts);

        ///////////// TF broadcast

    }//for msg size

     //Once all the information of the TFs is obtained, then we broadcast this data to Rviz
    broadcaster.sendTransform(v_ts);
    
  } // callback

    

}; // Class 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "direct_percept_node");
    ros::NodeHandle nh;

    DirectWorldInfo myDirectWorld(nh);

    ros::spin();

    return 0;
}