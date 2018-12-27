#ifndef LEG_JOINT_TRAJECTORY_ACTION_CONTROLLER_H__
#define LEG_JOINT_TRAJECTORY_ACTION_CONTROLLER_H__

#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <ros/node_handle.h>

#include <actionlib/server/action_server.h>
#include <filters/filter_chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ardent_controllers_msgs/JointTrajectoryState.h>
#include <ardent_controllers_msgs/JointTrajectoryControllerState.h>
#include <ardent_controllers_msgs/JointTrajectoryAction.h>
// ignore all of this, its for managing the controllers
template <class Action> class RTServerHandle
{
    private:
        ACTION_DEFINITION(Action);  // define the action namespace
        typedef actionlib::ServerGoalHandle<Action> GoalHandle;
        typedef boost::shared_ptr<Feedback> FeedbackPtr;
        typedef boost::shared_ptr<feedback> FeedbackPtr;

        unit8_t state;
        bool req_abort;
        bool req_succeed;
        ResultConstPtr req_result;

        GoalHandle gh;
        // reallocate result and feedback ptrs so it can be realtime
        ResultPtr rt_prealloc_result;
        FeedbackPtr rt_prealloc_feedback;
    public:

        RTServerHandle(GoalHandle &gh_, const ResultPtr &rt_prealloc_result_ = ResultPtr((Result*)NULL)
            : req_abort(false), req_succeed(false), gh(gh_), rt_prealloc_result(rt_prealloc_result)
        {
            // make sure the preallocated result is valid or give it a default value
            if(!rt_prealloc_result) 
            {
                rt_prealloc_result.reset(new Result);
            }
            if(!rt_prealloc_feedback)
            {
                rt_prealloc_feedback.reset(new Feedback);
            }
        }
}





#endif LEG_JOINT_TRAJECTORY_ACTION_CONTROLLER_H__