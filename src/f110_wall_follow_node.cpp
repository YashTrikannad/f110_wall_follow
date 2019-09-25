#include "f110_wall_follow/utility.h"

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

class WallFollow
{
public:
    WallFollow():
            node_handle_(ros::NodeHandle()),
            lidar_sub_(node_handle_.subscribe("scan", 5, &WallFollow::scan_callback, this)),
            drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 5))
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
        node_handle_.getParam("/error_based_velocities", error_based_velocities_);
        node_handle_.getParam("/truncated_coverage_angle", truncated_coverage_angle_);
        node_handle_.getParam("/smoothing_filter_size", smoothing_filter_size_);
    }

    std::vector<double> preprocess_scan(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        auto truncated_ranges = wf::truncate(scan_msg, truncated_coverage_angle_);
        for(auto& range : truncated_ranges)
        {
            if(std::isnan(range))
            {
                range = 0;
            }
        }
        return wf::apply_smoothing_filter(truncated_ranges, smoothing_filter_size_);
    }

    /// Returns the distance from obstacle at a given angle from the Laser Scan Message
    /// @param scan_msg - Laser Scan Message
    /// @param angle - Angle in Radians (0 rads -> right in front of the Car)
    /// @return
    double get_range_at_angle(const std::vector<double> &filtered_scan, const double& angle,
            const double angle_increment) const
    {
        const double corrected_angle = angle + (truncated_coverage_angle_/2);
        ROS_DEBUG("Corrected Angle : %f", corrected_angle);

        const double required_range_index = static_cast<int>(floor(corrected_angle/angle_increment));
        ROS_DEBUG("Required Range Index : %f", required_range_index);

        ROS_DEBUG("Required Range Value : %f", filtered_scan[required_range_index]);
        return filtered_scan[required_range_index];
    }

    /// PID controller to control the steering of the car and adjust the velocity accordingly
    void control_steering()
    {
        prev_reading_time_ = current_reading_time;
        current_reading_time = ros::Time::now().toSec();
        const auto dt = current_reading_time - prev_reading_time_;

        integral_ += error_;

        double steering_angle = kp_ * error_ + kd_ * (error_ - prev_error_) / dt + ki_ * (integral_);

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";

        if(std::isnan(steering_angle))
        {
            drive_msg.drive.speed = 0;
            drive_msg.drive.steering_angle = 0;
            std::__throw_runtime_error("The Control Value to Steering cannot be nan");
        }

        // Thresholding for limiting the movement of car wheels to avoid servo locking
        if(steering_angle > 0.4)
        {
            steering_angle = 0.4;
        }
        else if(steering_angle < -0.4)
        {
            steering_angle = -0.4;
        }

        ROS_DEBUG("Steering Angle : %f", steering_angle);
        drive_msg.drive.steering_angle = steering_angle;

        if(abs(steering_angle) > 0.349)
        {
            drive_msg.drive.speed = error_based_velocities_["high"];
        }
        else if(abs(steering_angle) > 0.174)
        {
            drive_msg.drive.speed = error_based_velocities_["medium"];
        }
        else
        {
            drive_msg.drive.speed = error_based_velocities_["low"];
        }
        drive_pub_.publish(drive_msg);

        prev_error_ = error_;
    }

    /// Returns value of Error between the required distance and the current distance
    /// @param scan_msg
    /// @param left_distance
    /// @return
    void get_error(const std::vector<double> &filtered_ranges, const double angle_increment)
    {
        const auto distance_of_a = get_range_at_angle(filtered_ranges, 0.5, angle_increment);
        const auto distance_of_b = get_range_at_angle(filtered_ranges, 1.4, angle_increment);
        constexpr auto theta = 0.9;

        const auto alpha = std::atan2(distance_of_a*cos(theta)-distance_of_b,distance_of_a*sin(theta));
        ROS_DEBUG("alpha: %f", alpha);

        const auto distance_t = distance_of_b*cos(alpha);
        const auto distance_tplus1 = distance_t + lookahead_distance_*sin(alpha);

        error_ = distance_tplus1 - desired_left_wall_distance_ ;
    }

    /// Scan Callback Function
    /// @param scan_msg
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        const auto filtered_ranges = preprocess_scan(scan_msg);
        get_error(filtered_ranges, scan_msg->angle_increment);
        control_steering();
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber lidar_sub_;
    ros::Publisher drive_pub_;

    double kp_, ki_, kd_;
    double prev_error_, error_;
    double integral_;

    double prev_reading_time_;
    double current_reading_time;

    double desired_left_wall_distance_;
    double lookahead_distance_;
    double truncated_coverage_angle_;

    int smoothing_filter_size_;
    std::map<std::string, double> error_based_velocities_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "wall_follow_node");
    WallFollow wall_follower;
    ros::spin();
    return 0;
}
