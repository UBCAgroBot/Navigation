#include <chrono>
#include <functional>
#include <memory>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class EKFPublisher : public rclcpp::Node
{
  public:
    EKFPublisher() : Node("ekf_publisher")
    {
      imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      
      

      timer_ = this->create_wall_timer(
      10ms, std::bind(&EKFPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        sensor_msgs::msg::Imu msg_imu;
        
        msg_imu.header.stamp = this->get_clock()->now();

        //set the frame to base_link, which is the frame that actually moves with the robot
        msg_imu.header.frame_id = "base_link";
        
        msg_imu.angular_velocity.x = 0; 
        msg_imu.angular_velocity.y = 0;
        msg_imu.angular_velocity.z = 0;


        //intialiaze orientiation, sinme it is not set to false in YAML config, u need to set manually
        msg_imu.orientation.x = 0.0;
        msg_imu.orientation.y = 0.0;
        msg_imu.orientation.z = 0.0;
        msg_imu.orientation.w = 1.0;
        
        // Not needed if set false in ekf.yaml
        // msg_imu.linear_acceleration.x = 0;
        // msg_imu.linear_acceleration.y = 0;
        // msg_imu.linear_acceleration.z = 0;
        
        imu_pub_->publish(msg_imu);

        nav_msgs::msg::Odometry msg_odom;

        msg_odom.header.stamp = this->get_clock()->now();

        //this basically a defines a transformatiuon, the odom establishes the odom frame, and then transofrms to the base_link frame
        msg_odom.header.frame_id = "odom";
        msg_odom.child_frame_id = "base_link";
        
        // msg_odom.twist.angular.x = 0; 
        // msg_odom.twist.angular.y = 0;
        // msg_odom.twist.angular.z = 0;
    
        // fixed speed
        msg_odom.twist.twist.linear.x = 1;
        msg_odom.twist.twist.linear.y = 0;
        msg_odom.twist.twist.linear.z = 0;

        x_pos_ += msg_odom.twist.twist.linear.x * 0.01;

        msg_odom.pose.pose.position.x = x_pos_;
        msg_odom.pose.pose.position.y = 0.0;
        
        odom_pub_->publish(msg_odom);
    }

    double x_pos_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

    // create EKF publisher
  rclcpp::spin(std::make_shared<EKFPublisher>());

  rclcpp::shutdown();
  return 0;
}
