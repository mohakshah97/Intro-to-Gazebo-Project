#include "geometry_msgs/Twist.h"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <my_rb1_ros/Rotate.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// Import the service message header file generated from the Empty.srv message
double _current_yaw ;
double target_yaw;
// We define the callback function of the service
class RotateServer {
public:
    RotateServer() {
        
        _server = _nh.advertiseService("/rotate_robot", &RotateServer::rotateCallback, this);
        _server_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
        _odom_sub = _nh.subscribe("/odom", 1, &RotateServer::odomCallback,this);
        
    }

    bool rotateCallback(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res) {
        ROS_INFO("Service Requested");
        double target_yaw = req.degrees * M_PI / 180.0;
        _old_yaw = _current_yaw;
        rotation = 0;
        double rotation_com = 0;
        target_yaw = abs(ceil(target_yaw*100)/100);
        while (rotation_com != target_yaw){
            //if (rotation == 0){_old_yaw = _current_yaw;}
            move_robot(req.degrees);
            rotation = rotation  + abs(_old_yaw - _current_yaw);
            rotation_com = (ceil(rotation*100)/100);
            //ROS_INFO("current yaw value is: %f and old_yaw is :%f and rotation is :%f",_current_yaw,_old_yaw, rotation);
            //r.sleep();
            _old_yaw = _current_yaw;
            ros::spinOnce();

        }
        
        stop_robot();
    
        

        // Implement a control loop to reach the target yaw
        // ... (e.g., using a PID controller)
        // ...
        res.result = "Rotation completed successfully";
        ROS_INFO("Service Completed");
        return true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        //_current_yaw = msg->pose.pose.orientation.z;
        tf2::Quaternion q(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        _current_yaw = abs(yaw);
        //ROS_INFO("current yaw value is: %f in subscriber callback",_current_yaw);
        //ROS_INFO("current z value is: %f in subscriber callback", msg->pose.pose.orientation.z);
    }

    void move_robot(int deg){

        if (deg > 0){
            rotate_msg.angular.z = 0.05;
            _server_pub.publish(rotate_msg);
        }
        else {
        rotate_msg.angular.z = - 0.05;
        _server_pub.publish(rotate_msg);
        }
    }

    void stop_robot(void){
        rotate_msg.linear.x = 0.0;
        rotate_msg.linear.y = 0.0;
        rotate_msg.angular.z = 0.0;
        _server_pub.publish(rotate_msg);
    }

private:
    ros::NodeHandle _nh;
    ros::ServiceServer _server;
    ros::Subscriber _odom_sub;
    ros::Publisher _server_pub;
    geometry_msgs::Twist rotate_msg;
    nav_msgs::Odometry odometry_data;
    double rotation = 0.0;
    double _old_yaw ;
    ros::Rate r = 1;
    //double _current_yaw = 0.0;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_service");
    RotateServer rotate_server;
    rotate_server.stop_robot();
    ROS_INFO("Service Ready");
    ros::spin();
    return 0;
}