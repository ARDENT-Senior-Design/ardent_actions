#include <ros/ros.h>
#include <vector>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ardent_controllers_msgs/JointTrajectoryAction.h>
#include <ardent_controllers_msgs/JointTrajectoryControllerState.h>

#define GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
    private:
        typedef actionlib::ActionServer<ardent_controllers_msgs::JointTrajectoryAction> JTAS;
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

        ardent_controllers_msgs::JointTrajectoryControllerStateConstPtr last_controller_state;

        std::string leg_id;

        JointTrajectoryExecuter(std::string leg_id_, ros::NodeHandle &nh_) :  
            node(nh_), 
            action_server(node,"joint_trajectory_action_server",
                            boost::bind(&JointTrajectoryExecuter::goalCallback, this, _1),
                            boost::bind(&JointTrajectoryExecuter::cancelCallback, this, _1),
                            false),
            has_active_goal(false)
        {
            using namespace XmlRpc;
            leg_id = leg_id_;
            ros::NodeHandle pn("leg_"+leg_id+"_traj_controller/");
            // resolves "~/gripper_action" to "/grasping_points_node/gripper_action"
            // read joints from the robot xml
            XmlRpc::XmlRpcValue joint_names_;
            if(!pn.getParam("joints", joint_names_))
            {
                ROS_FATAL("No joints given. (namespace: %s)", pn.getNamespace().c_str());
                exit(1);
            }
            if(joint_names_.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_FATAL("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
                exit(1);
            }
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                XmlRpcValue &name_value = joint_names_[i];
                if(name_value.getType() != XmlRpcValue::TypeString)
                {
                    ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)", pn.getNamespace().c_str());
                    exit(1);
                }
                else
                {
                    ROS_INFO("Added %s %s to the list of joints", pn.getNamespace().c_str(),(std::string)name_value);
                }
                joint_names.push_back((std::string)name_value);
            }
            pn.param("constraints/goal_time", goal_time_constraint, 0.0);
            for(size_t i =0; i < joint_names.size();++i)
            {
                std::string ns = std::string("constraints/")+joint_names[i];
                double goal, traj;
                pn.param(ns+"/goal",goal,-1.0);
                pn.param(ns+"/trajectory",traj,-1.0);
                goal_constraints[joint_names[i]] = traj;
            }
            pn.param("constaints/stopped_velocity_tolerance",stopped_velocity_tolerance, 0.01);

            pub_controller_command = node.advertise<trajectory_msgs::JointTrajectory>("command",1);
            sub_controller_state = node.subscribe("state", 1, &JointTrajectoryExecuter::controllerStateCallback, this);
        }

        ~JointTrajectoryExecuter()
        {
            pub_controller_command.shutdown();
            sub_controller_state.shutdown();
            //watchdog_timer.stop();
        }

        void watchdog(const ros::TimerEvent &e)
        {
            ros::Time now = ros::Time::now();

        }
        void goalCallback(GoalHandle gh_)
        {

        }

        void cancelCallback(GoalHandle gh_)
        {

        }

        void controllerStateCallback(const ardent_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)
        {

        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_action_node");
    ros::NodeHandle node;//("~");
    JointTrajectoryExecuter jte("rf",node);

    ros::spin();

    return 0;
}