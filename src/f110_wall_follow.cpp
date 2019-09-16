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
    {}

    double getRange(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const double& angle)
    {
        /*
        scan_msg: single message from topic /scan
        angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        Outputs length in meters to object with angle in lidar scan field of view
        make sure to take care of nans etc.
        */
        return 0.0;
    }

    void pid_control(const double error, const double velocity)
    {
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = 0;
        drive_msg.drive.speed = velocity;
        drive_pub_.publish(drive_msg);
    }

    double followLeft(const sensor_msgs::LaserScan::ConstPtr &scan_msg, double left_distance)
    {
        // left wall as per the algorithm
        return 0.0;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {

    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber lidar_sub_;
    ros::Publisher drive_pub_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "wall_follow_node");
    WallFollow wall_follower;
    ros::spin();
    return 0;
}
