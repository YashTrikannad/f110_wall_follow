#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

class WallFollow
{
public:
    WallFollow():
            node_handle_(ros::NodeHandle()),
            lidar_sub_(node_handle_.subscribe("scan", 100, &WallFollow::scan_callback, this)),
            drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 100))
    {
        node_handle_.getParam("/kp", kp_);
        node_handle_.getParam("/ki", ki_);
        node_handle_.getParam("/kd", kd_);
        prev_error_ = 0.0;
        error_ = 0.0;
        integral_ = 0.0;
        node_handle_.getParam("/desired_distance_left", desired_left_wall_distance_);
        node_handle_.getParam("/lookahead_distance", lookahead_distance_);
        prev_reading_time_ = ros::Time::now().toNSec();
        current_reading_time = ros::Time::now().toNSec();
    }

    /// Returns the distance from obstacle at a given angle from the Laser Scan Message
    /// @param scan_msg - Laser Scan Message
    /// @param angle - Angle in Radians (0 rads -> right in front of the Car)
    /// @return
    double get_range_at_angle(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const double& angle) const
    {
        const double corrected_angle = angle + 3.14;
        const double required_range_index = corrected_angle*scan_msg->ranges.size()/6.28;
        return scan_msg->ranges[static_cast<int>(required_range_index)];
    }

    void pid_control()
    {
        prev_reading_time_ = current_reading_time;
        current_reading_time = ros::Time::now().toSec();
        const auto dt = current_reading_time - prev_reading_time_;

        integral_ += error_*dt;

        const double p_control_value = kp_*error_ + kd_*(error_ - prev_error_)/dt + ki_*(integral_);

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = p_control_value;
        drive_msg.drive.speed = 0.5;
        drive_pub_.publish(drive_msg);

        prev_error_ = error_;
    }

    /// Returns value of Error between the required distance and the current distance
    /// @param scan_msg
    /// @param left_distance
    /// @return
    void get_error(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        const auto distance_of_a = get_range_at_angle(scan_msg, 0.7853);
        const auto distance_of_b = get_range_at_angle(scan_msg, 0.7853*2);
        constexpr auto theta = 0.7853;

        const auto alpha = std::atan2(distance_of_a*cos(theta)-distance_of_b,distance_of_a*sin(theta));

        const auto distance_t = distance_of_b*cos(alpha);
        const auto distance_tplus1 = distance_t + lookahead_distance_*sin(alpha);

        error_ = distance_tplus1 - desired_left_wall_distance_ ;
    }

    /// Scan Callback Function
    /// @param scan_msg
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        get_error(scan_msg);
        pid_control();
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber lidar_sub_;
    ros::Publisher drive_pub_;
    double kp_, ki_, kd_;
    double prev_error_, error_;
    double integral_;
    double desired_left_wall_distance_;
    double lookahead_distance_;
    double prev_reading_time_;
    double current_reading_time;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "wall_follow_node");
    WallFollow wall_follower;
    ros::spin();
    return 0;
}
