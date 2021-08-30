/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base
{
    //typedefs to help us out with the action server so that we don't hace to type so much
    // 1、声明server端，消息类型是move_base_msgs::MoveBaseAction
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

    // 2、枚举movebase状态表示
    enum MoveBaseState
    {
        PLANNING,    //在规划路径中
        CONTROLLING, //在控制机器人运动中
        CLEARING     //规划或者控制失败在恢复或者清除中。
    };
    // 一般默认状态或者接收到一个有效goal时是PLANNING，/在规划出全局路径后state_会由PLANNING->CONTROLLING，
    // 如果规划失败则由PLANNING->CLEARING。在MoveBase::executeCycle中，会分别对这三种状态做处理：
    //  还在PLANNING中则唤醒规划线程让它干活，如果已经在CONTROLLING中，判断是否已经到达目的地，否则判断是否出现震动？
    //   否则调用局部路径规划，如果成功得到速度则直接发布到cmd_vel，失败则判断是否控制超时，不超时的话让全局再规划一个路径。
    //   如果出现了问题需要CLEARING（仅有全局规划失败、局部规划失败、长时间困在一片小区域三种原因），
    //   则每次尝试一种recovery方法，直到所有尝试完。
    //movebase为recovery行为定义了如下三种原因

    // 3、枚举，触发恢复模式
    enum RecoveryTrigger
    {
        PLANNING_R,    //全局规划失败
        CONTROLLING_R, //局部规划失败
        OSCILLATION_R  //长时间困在一片小区域
    };

    // 4、MoveBase类，使用actionlib：：ActionServer接口，该接口将robot移动到目标位置
    /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
    class MoveBase
    {
    public:
        // 1、构造函数，传入的参数是tf
        /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
        MoveBase(tf2_ros::Buffer &tf);

        // 2、析构函数
        /**
       * @brief  Destructor - Cleans up
       */
        virtual ~MoveBase();
        // 3、控制闭环、全局规划、 到达目标返回true，没有到达返回false
        /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @param global_plan A reference to the global plan being used
       * @return True if processing of the goal is done, false otherwise
       */
        bool executeCycle(geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &global_plan);

    private:
        // 4、清除costmap
        /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
        bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
        // 5、当action不活跃时，调用此函数，返回plan
        /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
        bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

        // 6、新的全局规划，goal 规划的目标点，plan 路径
        /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
        bool makePlan(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);
        // 7、从参数服务器加载导航恢复行为
        /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
        bool loadRecoveryBehaviors(ros::NodeHandle node);
        // 8、加载默认导航恢复行为
        /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
        void loadDefaultRecoveryBehaviors();
        // 9、清除机器人局部规划框的障碍物，size_x 局部规划框的长x， size_y 局部规划框的宽y
        /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
        void clearCostmapWindows(double size_x, double size_y);
        // 10、发布速度为0的指令
        /**
       * @brief  Publishes a velocity command of zero to the base
       */
        void publishZeroVelocity();
        // 11、重置move_base action的状态，设置速度为0
        /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
        void resetState();
        // 13、其它函数
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal);

        void planThread();

        void executeCb(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal);

        bool isQuaternionValid(const geometry_msgs::Quaternion &q);

        bool getRobotPose(geometry_msgs::PoseStamped &global_pose, costmap_2d::Costmap2DROS *costmap);

        double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);

        geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg);

        // 12、周期性地唤醒规划器
        /**
       * @brief This is used to wake the planner at periodic intervals.
       */
        void wakePlanner(const ros::TimerEvent &event);

        // 14、数据成员
        tf2_ros::Buffer &tf_;

        MoveBaseActionServer *as_; //就是actionlib的server端

        boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;                        //局部规划器，加载并创建实例后的指针  //创建插件类实例 -- 参数:插件类全限定名称 //类加载器
        costmap_2d::Costmap2DROS *planner_costmap_ros_, *controller_costmap_ros_; //costmap的实例化指针

        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_; //全局规划器，加载并创建实例后的指针
        std::string robot_base_frame_, global_frame_;

        std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>> recovery_behaviors_; //可能出错后的恢复
        std::vector<std::string> recovery_behavior_names_;
        unsigned int recovery_index_;

        geometry_msgs::PoseStamped global_pose_;
        double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
        double planner_patience_, controller_patience_;
        int32_t max_planning_retries_;
        uint32_t planning_retries_;
        double conservative_reset_dist_, clearing_radius_;
        ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
        ros::Subscriber goal_sub_;
        ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
        bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
        bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
        double oscillation_timeout_, oscillation_distance_;

        MoveBaseState state_;
        RecoveryTrigger recovery_trigger_;

        ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
        geometry_msgs::PoseStamped oscillation_pose_;
        //类加载器 -- 参数1:基类功能包名称 参数2:基类全限定名称
        //  以插件形式实现全局规划器、局部规划器和丢失时恢复规划器。
        //插件形式可以实现随时动态地加载C++类库，但需要在包中注册该插件，不用这个的话需要提前链接（相当于运行时加载）
        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
        pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
        pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

        //set up plan triple buffer //触发哪种规划器
        std::vector<geometry_msgs::PoseStamped> *planner_plan_; //保存最新规划的路径，传给latest_plan_
        std::vector<geometry_msgs::PoseStamped> *latest_plan_;  ///作为一个桥梁，在MoveBase::executeCycle中传递给controller_plan_
        std::vector<geometry_msgs::PoseStamped> *controller_plan_;

        //set up the planner's thread   //规划器线程
        bool runPlanner_;
        boost::recursive_mutex planner_mutex_;
        boost::condition_variable_any planner_cond_; //boost的一种结合了互斥锁的用法，可以使一个线程进入睡眠状态，然后在另一个线程触发唤醒。
        geometry_msgs::PoseStamped planner_goal_;    //通过这个值将goal在MoveBase::executeCb与MoveBase::planThread()之间传递
        boost::thread *planner_thread_;

        boost::recursive_mutex configuration_mutex_;
        dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;

        void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

        move_base::MoveBaseConfig last_config_;
        move_base::MoveBaseConfig default_config_;
        bool setup_, p_freq_change_, c_freq_change_;
        bool new_global_plan_;
    };
};
#endif
