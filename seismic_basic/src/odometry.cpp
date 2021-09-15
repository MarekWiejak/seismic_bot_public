/*
 * Copyright 2021, AstroCeNT

 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
 * OR OTHER DEALINGS IN THE SOFTWARE.

 * AstroCeNT: Particie Astrophysics Science and Technology Centre
 * Agreement MAB / 2018/7 dated 03/07/2018 (Umowa MAB/2018/7 z dn. 03.07.2018)
 * The project is co-financed by the European Regional Development Fund under the Intelligent Development Operational Program
 * (Projekt jest współfinansowany ze środków Europejskiego Funduszu Rozwoju Regionalnego w ramach Programu Operacyjnego Inteligentny Rozwój)
 * and also from the Foundation for Polish Science grant TEAM/2016-3/19.
 * 
 * Author: Marek Wiejak
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <seismic_basic/SetOdometry.h>

class Odometer
{
    private:
    ros::Time current_time;
    ros::Time last_time;
    double x;
    double y;
    double th;
    tf::TransformBroadcaster odom_broadcaster;

    public:

    Odometer(){
        double x = 0.0;
        double y = 0.0;
        double th = 0.0;
        current_time = ros::Time::now();
        last_time = ros::Time::now();
    }

    void callback(const geometry_msgs::Twist& msg){
        this->current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (msg.linear.x * cos(th) - msg.linear.y * sin(th)) * dt;
        double delta_y = (msg.linear.x * sin(th) + msg.linear.y * cos(th)) * dt;
        double delta_th = msg.angular.z * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom_frame";
        odom_trans.child_frame_id = "robot_frame";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        odom_broadcaster.sendTransform(odom_trans);

        last_time = current_time;
    }

    bool set_odometry(seismic_basic::SetOdometry::Request &req, seismic_basic::SetOdometry::Response &res){
        x = req.x;
        y = req.y;
        th = req.z;
        return true;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    Odometer odometer;
    ros::Subscriber vel_sub = n.subscribe("measured_vel", 1000, &Odometer::callback, &odometer);
    ros::ServiceServer service = n.advertiseService("set_odometry", &Odometer::set_odometry, &odometer);
    
    ros::spin();
    return 0;
    
}