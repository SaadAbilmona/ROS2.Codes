#include "line_control/line_control.h"

#include <cmath>

LineControl::LineControl() : Node("line_control_node")
{
    RCLCPP_INFO(this->get_logger(), "LineControl initialisation");

    this->declare_parameter("line_y", -10.0);
    this->declare_parameter("cx", -6.0);
    this->declare_parameter("cy", 0.0);
    this->declare_parameter("R", 6.0);
    this->declare_parameter("task_vel", 1.0);
    this->declare_parameter("prop_factor", 0.1);
    this->declare_parameter("int_factor", 0.0);
    this->declare_parameter("diff_factor", 0.0);
    this->declare_parameter("min_obstacle_range", 1.0);
    this->declare_parameter("dt", 0.1);

    this->get_parameter("line_y", line_y);
    this->get_parameter("cx", cx);
    this->get_parameter("cy", cy);
    this->get_parameter("R", R);
    this->get_parameter("task_vel", task_vel);
    this->get_parameter("prop_factor", prop_factor);
    this->get_parameter("int_factor", int_factor);
    this->get_parameter("diff_factor", diff_factor);
    this->get_parameter("min_obstacle_range", min_obstacle_range);
    double dt;
    this->get_parameter("dt", dt);

    // Update laser scan subscription for Gazebo
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 100, std::bind(&LineControl::laserCallback, this, std::placeholders::_1));

    // Update the odometry subscription for Gazebo
    pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 100, std::bind(&LineControl::poseCallback, this, std::placeholders::_1));

    timer1 = this->create_wall_timer(
        std::chrono::duration<double>(dt), std::bind(&LineControl::timerCallback, this));

    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);
    err_pub = this->create_publisher<std_msgs::msg::Float64>("/err", 100);
}

void LineControl::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const double kMinObstacleDistance = 0.3;
    for (auto r : msg->ranges)
    {
        if (r < kMinObstacleDistance)
        {
            obstacle = true;
            RCLCPP_WARN(this->get_logger(), "OBSTACLE!!!");
            break;
        }
    }
}

void LineControl::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    theta = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    RCLCPP_DEBUG(this->get_logger(), "Pose msg: x=%.2f y=%.2f theta=%.2f", x, y, theta);
}

double LineControl::cross_track_err_line() { return line_y - y; }
double LineControl::cross_track_err_line(double y_line) { return y_line - y; }
double LineControl::cross_track_err_circle() { return sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y)) - R; }
double LineControl::cross_track_err_circle(double c_x, double c_y, double radius)
{
    return sqrt((c_x - x) * (c_x - x) + (c_y - y) * (c_y - y)) - radius;
}

double LineControl::cross_track_err_oval()
{
    if (x > 6) return cross_track_err_circle(6, 0, 6);
    if (x < -6) return cross_track_err_circle(-6, 0, 6);
    if (y > 0) return -cross_track_err_line(6);
    return cross_track_err_line(-6);
}

void LineControl::publish_error(double e)
{
    std_msgs::msg::Float64 err;
    err.data = e;
    err_pub->publish(err);
}

void LineControl::timerCallback()
{
    geometry_msgs::msg::Twist cmd;
    if (!obstacle)
    {
        double err = cross_track_err_oval();
        publish_error(err);
        int_error += err;
        double diff_error = err - old_error;
        old_error = err;
        cmd.linear.x = task_vel;
        cmd.angular.z = prop_factor * err + int_factor * int_error + diff_error * diff_factor;
        RCLCPP_DEBUG(this->get_logger(), "error=%.2f cmd.v=%.2f w=%.2f", err, cmd.linear.x, cmd.angular.z);
    }
    cmd_pub->publish(cmd);
}
