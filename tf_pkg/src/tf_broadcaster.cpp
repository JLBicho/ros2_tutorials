#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class TfBroadcaster : public rclcpp::Node{
private:
    void objectCallback(const std_msgs::msg::Bool::SharedPtr msg){
        // All transforms must be correctly time-stamped!
        transf_.header.stamp = this->get_clock()->now();
        if(msg->data){
            transf_.transform.translation.x = 1.0;
            transf_.transform.translation.y = 1.0;
        }else{
            transf_.transform.translation.x = 2.0;
            transf_.transform.translation.y = 3.0;
        }
            tf2_broadcaster_->sendTransform(transf_);
    }

    // Declare the subscription to the simulated detected object topic
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr object_sub_;

    // Declare the transforms broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

    // Declare the transform to broadcast
    geometry_msgs::msg::TransformStamped transf_;

public:
    TfBroadcaster(const std::string & name):
    Node(name){
        // Initialize the transforms broadcaster
        tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Define the "parent" frame
        transf_.header.frame_id = "sensor_link";
        // Define the "child" frame
        transf_.child_frame_id = "object_link";
        // Initialize the subscription to the simulated detected object topic
        object_sub_ = create_subscription<std_msgs::msg::Bool>("/object_detected", 10, std::bind(&TfBroadcaster::objectCallback,this,std::placeholders::_1));
    }

    ~TfBroadcaster(){}
};

int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfBroadcaster>("tf_broadcaster");
    rclcpp::spin(node);
    return 0;
}
