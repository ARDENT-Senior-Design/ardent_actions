#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ardent_controllers_msgs/JointTrajectoryAction.h>
#include <ardent_controllers_msgs/JointTrajectoryControllerState.h>

#define GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecutor
{
    private:
        typedef actionlib::ActionServer<ardent_controllers_msgs::JointTrajectoryAction> JTAS
        typedef JTAS::GoalHandle GoalHandle;
    public:
        ros::NodeHandle node;
        JTAS action_server;
        ros::Publisher pub_controller_command;
        ros::Subscriber sub_controller_state;
        ros::Timer watchdog_timer;

        bool has_active_goal;
        GoalHandle active_goal;
        trajectory_msgs::JointTrajectory current_traj;


        std::vector<std::string> joint_names;
        std::map<std::string,double> goal_constraints;
        std::map<std::string,double> trajectory_constraints;
        double goal_time_constraint;
        double stopped_velocity_tolerance;

        pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr last_controller_state;

        JointTrajectoryExecutor(ros::NodeHandle &nh_) :  
            node(nh_), 
            action_server(node,"joint_trajectory_action",
                            boost::bind(&JointTrajectoryExecutor::goalCallback, this, _1),
                            boost::bind(&JointTrajectoryExecutor::cancelCallback, this, _1),
                            false),
            has_active_goal(false)
        {
            using namespace XmlRpc;
            ros::NodeHandle pn("~");

            // read joints from the robot xml
            XmlRpc::XmlRpcValue joint_names;
            if(!pn.getParam("joints", joint_names))
            {
                ROS_FATAL("No joints given. (namespace: %s)", pn.getNamespace().c_str());
                exit(1);
            }
            if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_FATAL("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
                exit(1);
            }
            for (int i = 0; i < joint_names.size(); ++i)
            {
                XmlRpcValue &name_value = joint_names[i];
            if(name_value.getType() != XmlRpcValue::TypeString)
            {
                ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)",
                        pn.getNamespace().c_str());
                exit(1);
            }

            joint_names_.push_back((std::string)name_value);
            }
        }

        void goalCallback(GoalHandle gh_)
        {

        }

        void cancelCallback(GoalHandle gh_)
        {

        }

        void controllerStateCallback(const ardent_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)
}