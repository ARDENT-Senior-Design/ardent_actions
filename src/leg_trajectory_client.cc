#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ardent_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

// This file will actually execute a specified leg parabola action based on: 
// http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
typedef actionlib::SimpleActionClient<ardent_controllers_msgs::JointTrajectoryAction> Client;

class LegParabola
{
    private:
        Client* traj_client;
        std::string leg_id;
    public:
        LegParabola(std::string leg_id_)
        {
            leg_id = leg_id_;
            //spin thread by default
            //defines the namespace and target leg_action_server, where the info will be published
            traj_client = new Client("leg_"+leg_id+"_traj_controller/leg_action_server",true);
            while(!traj_client->waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the joint_trajectory_action server");
            }
        }

        ~LegParabola(){ delete traj_client;}

        // Send the command to start a given trajectory goal
        void startTrajectory(ardent_controllers_msgs::JointTrajectoryGoal goal)
        {
            // Send the trajectory to follow
            goal.trajectory.header.stamp = ros::Time::now();
            traj_client->sendGoal(goal);
        } 

        /*
            Generate a simple trajectory containing two waypoints
        */
        ardent_controllers_msgs::JointTrajectoryGoal createLegTrajectory()
        {
            ardent_controllers_msgs::JointTrajectoryGoal goal;

            // Joint names that apply to waypoint list
            goal.trajectory.joint_names.push_back("j_femur_"+leg_id);
            goal.trajectory.joint_names.push_back("j_coxa_"+leg_id);
            goal.trajectory.joint_names.push_back("j_tibia_"+leg_id);
            
            // 2 waypoints are in the list
            goal.trajectory.points.resize(2);

            // First trajectory waypoint poisition for each joint is 0
            int i =0;
            goal.trajectory.points[i].positions[0] = 0.0; 
            goal.trajectory.points[i].positions[1] = 0.0;
            goal.trajectory.points[i].positions[2] = 0.0;
            for(int j=0;j<goal.trajectory.points.size();i++)
            {
                goal.trajectory.points[i].velocities[j] = 0.0;
                goal.trajectory.points[i].accelerations[j] = 0.0;
            }
            // 1st Waypoint goal should be reached 1 second after starting along trajectory
            goal.trajectory.points[i].time_from_start = ros::Duration(1.0);
            i++;
            // Create second waypoints so each joint goes to 0.25
            goal.trajectory.points[i].positions[0] = 0.25;
            goal.trajectory.points[i].positions[1] = 0.25;
            goal.trajectory.points[i].positions[2] = 0.25;
            for(int j=0;j<goal.trajectory.points.size();i++)
            {
                goal.trajectory.points[i].velocities[j] = 0.0;
                goal.trajectory.points[i].accelerations[j] = 0.0;
            }
            // 2nd Waypoint goal should be reached 2 seconds after starting along the trajectory
            goal.trajectory.points[i].time_from_start = ros::Duration(2.0);

            return goal;
        }

        actionlib::SimpleClientGoalState getState()
        {
            return traj_client->getState();
        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leg_trajectory_client");
    LegParabola leg("rf");
    ardent_controllers_msgs::JointTrajectoryGoal goalTraj = leg.createLegTrajectory();
    leg.startTrajectory(goalTraj);
    while(!leg.getState().isDone() && ros::ok())
    {
    }
    return 0;
}