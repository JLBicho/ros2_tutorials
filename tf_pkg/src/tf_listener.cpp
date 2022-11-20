#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class TfListener : public rclcpp::Node{
private:
    void timerCallback(){
        // IMPORTANT 
        // Use "try-catch" semantics to avoid problems when a transform does not exists
        try{
            // LookupTransform will operate all the matrices between the target_frame and the source_frame
            transf_ = tf2_buffer_->lookupTransform(
                "base_link", "object_link", tf2::TimePointZero);
            RCLCPP_INFO(get_logger(),"Transform between 'base_link'->'object_link' is:\n Position x,y,z = %.1f,%.1f,%.1f",
                transf_.transform.translation.x,
                transf_.transform.translation.y,
                transf_.transform.translation.z);
        }
        catch (const tf2::TransformException & ex){
            RCLCPP_WARN(get_logger(),"Could not transform between 'base_link'->'object_link'");
            return;
        }
    }
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    // Declare the listener
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    // Declare the buffer
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    // Transform message received
    geometry_msgs::msg::TransformStamped transf_;

public:
    TfListener(const std::string & name):
    Node(name){
        // Load a buffer of transforms
        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Make a transform listener of the buffer
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
        // Inicialize the timer that will check the transforms each second.
        timer_ = create_wall_timer(std::chrono::seconds(1), 
            std::bind(&TfListener::timerCallback, this));
    }
    ~TfListener(){

    }
};

int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfListener>("tf_listener");
    rclcpp::spin(node);
    return 0;
}
