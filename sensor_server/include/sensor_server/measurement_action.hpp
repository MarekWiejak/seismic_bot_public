#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_server/MeasurementAction.h>
    
namespace sensor_server{
    
    // an action server class
    class MeasurementActionServer {
        protected:
            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<sensor_server::MeasurementAction> as_;
            std::string action_name_;
            sensor_server::MeasurementFeedback feedback_;
            sensor_server::MeasurementResult result_;

        public:
            MeasurementActionServer(std::string name, ros::NodeHandle nh) :
                nh_(nh),
                as_(nh_, name, boost::bind(&MeasurementActionServer::executeCB, this, _1), false),
                action_name_(name)
            {
                as_.start();
            }

            ~MeasurementActionServer(void){}

            void executeCB(const sensor_server::MeasurementGoalConstPtr &goal);
        };
}