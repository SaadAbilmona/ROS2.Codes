#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class LineControl : public rclcpp::Node
{
public:
    LineControl();

private:
    double cross_track_err_line();
    double cross_track_err_line(double y_line);
    double cross_track_err_circle();
    double cross_track_err_circle(double c_x, double c_y, double radius);
    double cross_track_err_oval();

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void publish_error(double e);

    double line_y, cx, cy, R;
    double task_vel, prop_factor, int_factor, diff_factor;
    double int_error, old_error, min_obstacle_range;
    bool obstacle = false;
    double x, y, theta;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    
    rclcpp::TimerBase::SharedPtr timer1;
};

#endif  // LINE_CONTROL_H
