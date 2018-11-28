#include <ros/ros.h>
#include <ardent_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<ardent_controllers_msgs::JointTrajectoryAction> Client;

class LegParabola
{
    private:
        Client* traj_client;
    public:
        LegParabola(std::string leg_id)
        {
            //spin thread by default
            //defines the namespace and target leg_action_server, where the info will be published
            traj_client = new Client("leg_"+leg_id+"_traj_controller/leg_action_server",true);
            while(!traj_client->waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the joint_trajectory_action server")
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leg_trajectory_client");
    LegParabola leg("rf");
    leg.startTrajectory();
    return 0;
}