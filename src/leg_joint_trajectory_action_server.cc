#include <ros/ros.h>
#include <vector>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ardent_controllers_msgs/JointTrajectoryAction.h>
#include <ardent_controllers_msgs/JointTrajectoryControllerState.h>

#define GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
    // This class just monitors and manages the controller JointTrajectoryActionController in ardent_controllers
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
            action_server(node,"leg_rf_traj_controller/joint_trajectory_action",
                            boost::bind(&JointTrajectoryExecuter::goalCallback, this, _1),
                            boost::bind(&JointTrajectoryExecuter::cancelCallback, this, _1),
                            false),
            has_active_goal(false)
        {
            using namespace XmlRpc;
            leg_id = leg_id_;
            // trajectory namespace node handle
            ros::NodeHandle pn("leg_rf_traj_controller");
            //"leg_rf_traj_controller"
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
            /*
            j_coxa_rf
            j_femur_rf
            j_tibia_rf
            */
           //hardcoded from a rosparam get /leg_rf_traj_controller/joints
            joint_names_[0] = "j_coxa_rf";
            joint_names_[1] = "j_femur_rf";
            joint_names_[2] = "j_tibia_rf";

            for (int i = 0; i < joint_names_.size(); ++i)
            {
                XmlRpcValue &name_value = joint_names_[i];
                if(name_value.getType() != XmlRpcValue::TypeString)
                {
                    ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)", pn.getNamespace().c_str());
                    exit(1);
                }
                else
                {
                    ROS_INFO("Added %s %s to the list of joints", pn.getNamespace().c_str(),static_cast<std::string>(name_value).c_str());
                }
                joint_names.push_back(static_cast<std::string>(name_value).c_str());
            }
            pn.param("constraints/goal_time", goal_time_constraint, 0.0);
            for(size_t i =0; i < joint_names.size();++i)
            {
                std::string ns = std::string("constraints/")+joint_names[i];
                double goal, traj;
                pn.param(ns+"/goal",goal,-1.0);
                pn.param(ns+"/trajectory",traj,-1.0);
                goal_constraints[joint_names[i]] = goal;
                trajectory_constraints[joint_names[i]] = traj;
            }
            pn.param("constaints/stopped_velocity_tolerance",stopped_velocity_tolerance, 0.01);

            // publish the joint trajectory command to the robot_controllers
            pub_controller_command = node.advertise<trajectory_msgs::JointTrajectory>("command",1);
            // get the state information from the robot_controllers
            sub_controller_state = node.subscribe("state", 1, &JointTrajectoryExecuter::controllerStateCallback, this);

            watchdog_timer = node.createTimer(ros::Duration(1.0), &JointTrajectoryExecuter::watchdog, this);
            ros::Time started_waiting_for_controller = ros::Time::now();
            while(ros::ok() && !last_controller_state)
            {
                ros::spinOnce();
                if(started_waiting_for_controller != ros::Time(0) && ros::Time::now() > started_waiting_for_controller+ros::Duration(30.0))
                {
                    ROS_WARN("I've been waiting for 30 freaking seconds, where is my controller?");
                    started_waiting_for_controller = ros::Time(0);
                }
                ros::Duration(0.1).sleep();
            }

            // actually make the server visible to client after all these checks
            action_server.start();
        }

        ~JointTrajectoryExecuter()
        {
            pub_controller_command.shutdown();
            sub_controller_state.shutdown();
            //watchdog_timer.stop();
        }

        static bool checkArrayEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
        {
            if(a.size() != b.size())
            {
                return false;
            }
            for(size_t i=0; i < a.size();++i)
            {
                if(count(b.begin(), b.end(),a[i]) != 1)
                {
                    return false;
                }
            }
            for(size_t i=0; i < b.size();++i)
            {
                if(count(a.begin(),a.end(),b[i]) != 1)
                {
                    return false;
                }
            }
            return true;
        }

        void watchdog(const ros::TimerEvent &e)
        {
            ros::Time now = ros::Time::now();
            //if the controller is not active, abort
            if(has_active_goal)
            {
                bool should_abort = false;
                if(!last_controller_state)
                {
                    should_abort = true;
                    ROS_WARN("Abort because we haven't heard a controller message");
                }
                else if((now-last_controller_state->header.stamp) > ros::Duration(5.0))
                {
                    should_abort = true;
                    ROS_WARN("Abort because we haven't heard from the controller in %.31f seconds", (now-last_controller_state->header.stamp).toSec());
                }
                if(should_abort)
                {
                    // stop the controlelr
                    trajectory_msgs::JointTrajectory emptyTraj;
                    emptyTraj.joint_names = joint_names;
                    pub_controller_command.publish(emptyTraj);

                    // Abort the current goal
                    active_goal.setAborted();
                    has_active_goal = false;
                }
            }
        }
        void goalCallback(GoalHandle gh_)
        {
            // Verifies and ensures that the joints in the goal match the joints being commanded
            if(!checkArrayEqual(joint_names, gh_.getGoal()->trajectory.joint_names))
            {
                ROS_ERROR("Goal joints and our joints don't match");
                gh_.setRejected();
                return;
            }

            // Cancels the active goal if there is already a goal and stops the trajectory
            if(has_active_goal)
            {
                //stop the controller
                trajectory_msgs::JointTrajectory emptyTraj;
                emptyTraj.joint_names = joint_names;
                pub_controller_command.publish(emptyTraj);
                active_goal.setCanceled();
                has_active_goal = false;
                ROS_WARN("Canceled active trajectory");
            }

            gh_.setAccepted();
            active_goal = gh_;
            has_active_goal = true;

            current_traj = active_goal.getGoal()->trajectory;
            pub_controller_command.publish(current_traj);
        }

        void cancelCallback(GoalHandle gh_)
        {
            if(active_goal == gh_)
            {
                //stop the controller
               trajectory_msgs::JointTrajectory emptyTraj;
                emptyTraj.joint_names = joint_names;
                pub_controller_command.publish(emptyTraj);
                active_goal.setCanceled();
                has_active_goal = false;
                ROS_WARN("Canceled active trajectory");
            }
        }

        void controllerStateCallback(const ardent_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)
        {
            last_controller_state = msg;
            ros::Time now = ros::Time::now();

            if(!has_active_goal)
            {
                return;
            }
            if(current_traj.points.empty())
            {
                return;
            } 
            if(now < current_traj.header.stamp+current_traj.points[0].time_from_start)
            {
                return;
            }
            if(!checkArrayEqual(joint_names,msg->joint_names))
            {
                ROS_ERROR("Joint names from the controller don't match our joint names");
                return;
            }

            int last = current_traj.points.size() - 1;
            ros::Time end_time = current_traj.header.stamp + current_traj.points[last].time_from_start;

            // Verify the controller is within constraints of the trajectory
            if(now < end_time)
            {
                // Check that the controller is inside the trajectory constraints for each joint
                for(size_t i = 0; i<msg->joint_names.size();++i)
                {   
                    // check the error 
                    double abs_error = fabs(msg->error.positions[i]);
                    // check the constraint at current trajectory point
                    double constraint = trajectory_constraints[msg->joint_names[i]];
                    if(constraint > 0 && abs_error > constraint)
                    {
                        // Stop the controller
                        trajectory_msgs::JointTrajectory emptyTraj;
                        emptyTraj.joint_names = joint_names;
                        pub_controller_command.publish(emptyTraj);
                        active_goal.setCanceled();
                        has_active_goal = false;
                        ROS_WARN("Canceled active trajectory. Outside trajectory constraints");
                        return;
                    }
                    else
                    {
                        // Check that we ended inside the goal constraints
                        bool inside_goal_constraints = true;
                        for(size_t i=0; i < msg->joint_names.size() && inside_goal_constraints;i++)
                        {
                            double abs_error = fabs(msg->error.positions[i]);
                            double goal_constraint = goal_constraints[msg->joint_names[i]];
                            if(goal_constraint >= 0 && abs_error > goal_constraint)
                            {
                                inside_goal_constraints = false;
                            }
                            
                            // I'm going to say we stopp now
                            if(fabs(msg->target.velocities[i]) < 1e-6)
                            {
                                if(fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance)
                                {
                                    inside_goal_constraints = false;
                                }
                            }
                            if(inside_goal_constraints)
                            {
                                active_goal.setSucceeded();
                                has_active_goal = false;
                            }
                            else if (now < end_time + ros::Duration(goal_time_constraint))
                            {
                                // Still have some time left to make it.
                            }
                            else
                            {
                                ROS_WARN("Aborting because we aren't inside the goal constraints");
                                active_goal.setAborted();
                                has_active_goal = false;
                            }
                        }
                    }
                }
            }
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