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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base
{
    //1、构造函数，传入的参数tf为Buffer的对象的引用。初始化一些成员
    MoveBase::MoveBase(tf2_ros::Buffer &tf) : tf_(tf),                                                                                                 //tf2_ros::Buffer& 引用？取址
                                              as_(NULL),                                                                                               //MoveBaseActionServer* 指针
                                              planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),                                               //costmap的实例化指针
                                              bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),                                                  //nav_core::BaseGlobalPlanner类型的插件  //类加载器 -- 参数1:基类功能包名称 参数2:基类全限定名称
                                              blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),                                                   //nav_core::BaseLocalPlanner类型的插件
                                              recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),                                              //nav_core::RecoveryBehavior类型的插件
                                              planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),                                         //三种规划器，看触发哪种
                                              runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) //配置参数
    {
        ////就是actionlib的server端 创建move_base action,绑定回调函数。
        // 维护MoveBase的MoveBaseActionServer状态机，并且新建了一个executeCb回调线程。这里由action的客户端触发回调线程。
        as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
        ROS_INFO("Hello_world-2-  as_");
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        recovery_trigger_ = PLANNING_R; //触发模式（三种模式：规划、控制、振荡）设置为“规划中”：

        //get some parameters that will be global to the move base node
        //2、从参数服务器获取一些参数，包括两个规划器名称、代价地图坐标系、规划频率、控制周期等
        std::string global_planner, local_planner;
        private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS")); //navfn::NavfnROS global_planner::GlobalPlanner
        private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
        private_nh.param("planner_frequency", planner_frequency_, 0.0);
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        // 路径规划等待时间
        private_nh.param("planner_patience", planner_patience_, 5.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);
        private_nh.param("max_planning_retries", max_planning_retries_, -1); // disabled by default
        // 震荡时间与距离
        private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
        private_nh.param("oscillation_distance", oscillation_distance_, 0.5);
        // parameters of make_plan service
        private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
        private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

        //set up plan triple buffer
        //创建了指向geometry_msgs::PoseStamped数据格式的三个容器,用于路径规划的三重缓冲区
        //为三种规划器（planner_plan_保存最新规划的路径，传递给latest_plan_，然后latest_plan_通过executeCycle中传给controller_plan_）设置内存缓冲区：
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

        //set up the planner's thread 创建了一个新的线程，用于路径规划
        //3、新建planner线程，入口为MoveBase::planThread
        //设置规划器线程，planner_thread_是boost::thread*类型的指针：
        ROS_INFO("Hello_world-3 -planner_thread_");
        planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));
        ROS_INFO("Hello_world-4 - planner_thread_");

        //for commanding the base
        //创建发布者，话题名一个是cmd_vel，一个是cunrrent_goal，一个是goal：
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

        ros::NodeHandle action_nh("move_base");
        action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
        recovery_status_pub_ = action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

        //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
        //they won't get any useful information back about its status, but this is useful for tools like nav_view and rviz
        //提供消息类型为geometry_msgs::PoseStamped的发送goals的接口，cb为MoveBase::goalCB 在rviz中输入的目标点就是通过这个函数来响应的：
        ros::NodeHandle simple_nh("move_base_simple");
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

        //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
        //设置costmap参数，技巧是把膨胀层设置为大于机器人的半径
        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
        private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

        private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
        private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
        private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

        //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
        //设置全局路径规划器，planner_costmap_ros_是costmap_2d::Costmap2DROS*类型的实例化指针.
        ROS_INFO("Hello_world-5 - planner_costmap_ros_");
        planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause(); // 暂停执行
        ROS_INFO("Hello_world-6 - planner_costmap_ros_");

        //initialize the global planner
        //初始化global planner，包括查看规划器是否有效，通过代价地图创建实例等
        // pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
        //planner_是boost::shared_ptr<nav_core::BaseGlobalPlanner>类型：

        try
        {
            planner_ = bgp_loader_.createInstance(global_planner); ////global_planner::GlobalPlanner
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_->pause();

        //create a local planner
        //初始化local planner，包括查看规划器是否有效，通过代价地图创建实例等
        try
        {
            tc_ = blp_loader_.createInstance(local_planner); //这个时候，tc_就成了局部规划器的实例对象。
            ROS_INFO("Created local_planner %s", local_planner.c_str());
            tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
            exit(1);
        }

        // Start actively updating costmaps based on sensor data
        //开始更新costmap：
        planner_costmap_ros_->start();
        controller_costmap_ros_->start();

        //advertise a service for getting a plan
        //全局规划：定义一个名为make_plan的服务，cb为MoveBase::planService
        make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

        //advertise a service for clearing the costmaps
        //开始清除地图服务：定义一个名为clear_costmaps的服务，cb为MoveBase::clearCostmapsService 提供清除一次costmap的功能：
        clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        //如果不小心关闭了costmap， 则停用：
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }

        //load any user specified recovery behaviors, and if that fails load the defaults
        //加载指定的恢复器，加载不出来则使用默认的，这里包括了找不到路自转360：
        if (!loadRecoveryBehaviors(private_nh))
        {
            loadDefaultRecoveryBehaviors();
        }

        //initially, we'll need to make a plan
        //导航过程基本结束，把状态初始化：
        state_ = PLANNING;

        //we'll start executing recovery behaviors at the beginning of our list
        recovery_index_ = 0;

        //we're all set up now so we can start the action server
        ////10.开启move_base动作器 启动actionlib服务器
        as_->start();

        //启动动态参数服务器(回调函数为reconfigureCB
        dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
        dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }
    // 18、动动态参数服务器回调函数，用于配置动态参数
    void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level)
    {
        // ROS中的参数服务器无法在线动态更新，也就是说如果Listener不主动查询参数值，
        // 就无法获知Talker是否已经修改了参数。这就对ROS参数服务器的使用造成了很大的局限，
        // 很多场景下我们还是需要动态更新参数的机制，例如参数调试、功能切换等，
        // 所以ROS提供了另外一个非常有用的功能包——dynamic_reconfigure，实现这种动态配置参数的机制。
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        //The first time we're called, we just want to make sure we have the
        //original configuration
        // //一旦被调用，我们要确保有原始配置
        if (!setup_)
        {
            last_config_ = config;
            default_config_ = config;
            setup_ = true;
            return;
        }

        if (config.restore_defaults)
        {
            config = default_config_;
            //if someone sets restore defaults on the parameter server, prevent looping
            //如果有人在参数服务器上设置默认值，要防止循环
            config.restore_defaults = false;
        }

        if (planner_frequency_ != config.planner_frequency)
        {
            planner_frequency_ = config.planner_frequency;
            p_freq_change_ = true;
        }

        if (controller_frequency_ != config.controller_frequency)
        {
            controller_frequency_ = config.controller_frequency;
            c_freq_change_ = true;
        }

        planner_patience_ = config.planner_patience;
        controller_patience_ = config.controller_patience;
        max_planning_retries_ = config.max_planning_retries;
        conservative_reset_dist_ = config.conservative_reset_dist;

        recovery_behavior_enabled_ = config.recovery_behavior_enabled;
        clearing_rotation_allowed_ = config.clearing_rotation_allowed;
        shutdown_costmaps_ = config.shutdown_costmaps;

        oscillation_timeout_ = config.oscillation_timeout;
        oscillation_distance_ = config.oscillation_distance;
        if (config.base_global_planner != last_config_.base_global_planner)
        {
            boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
            //initialize the global planner
            ////创建全局规划
            ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
            try
            {
                planner_ = bgp_loader_.createInstance(config.base_global_planner);

                // wait for the current planner to finish planning // 等待当前规划结束
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

                // Clean up before initializing the new planner // 在新规划开始前clear旧的
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

                lock.unlock();
            }
            catch (const pluginlib::PluginlibException &ex)
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                          config.base_global_planner.c_str(), ex.what());
                planner_ = old_planner;
                config.base_global_planner = last_config_.base_global_planner;
            }
        }

        if (config.base_local_planner != last_config_.base_local_planner)
        {
            boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
            //create a local planner //创建局部规划
            try
            {
                tc_ = blp_loader_.createInstance(config.base_local_planner);
                // Clean up before initializing the new planner // 清理旧的
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
            }
            catch (const pluginlib::PluginlibException &ex)
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                          config.base_local_planner.c_str(), ex.what());
                tc_ = old_planner;
                config.base_local_planner = last_config_.base_local_planner;
            }
        }

        make_plan_clear_costmap_ = config.make_plan_clear_costmap;
        make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

        last_config_ = config;
    }
    //13、为rviz等提供调用借口，
    // 该回调函数将geometry_msgs::PoseStamped形式的goal转换成move_base_msgs::MoveBaseActionGoal，再发布到对应类型的goal话题中
    void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal)
    {
        ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;

        action_goal_pub_.publish(action_goal);
    }
    // 9、清除机器人局部规划框的障碍物，size_x 局部规划框的长x， size_y 局部规划框的宽y 提供清除一次costmap的功能：
    void MoveBase::clearCostmapWindows(double size_x, double size_y)
    {
        geometry_msgs::PoseStamped global_pose;

        //clear the planner's costmap
        getRobotPose(global_pose, planner_costmap_ros_);

        std::vector<geometry_msgs::Point> clear_poly;
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        geometry_msgs::Point pt;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

        //clear the controller's costmap
        getRobotPose(global_pose, controller_costmap_ros_);

        clear_poly.clear();
        x = global_pose.pose.position.x;
        y = global_pose.pose.position.y;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
    }
    // 4、清除costmap
    bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
    {
        //clear the costmaps
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
        //调用的是costmap包（注意！！外部链接！！！）该函数的功能是重置地图，内部包括重置总地图、重置地图各层：
        controller_costmap_ros_->resetLayers();

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
        planner_costmap_ros_->resetLayers();
        return true;
    }
    // 5、当action不活跃时，调用此函数，返回plan。写了全局规划的策略，以多少距离向外搜索路径
    bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
    {
        //  这是movebase提供的一个服务 这三个包的cb其实都是调用相应的全局规划器来获得一条path返回给客户端。
        if (as_->isActive())
        {
            ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
            return false;
        }
        //make sure we have a costmap for our planner
        if (planner_costmap_ros_ == NULL)
        {
            ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
            return false;
        }

        geometry_msgs::PoseStamped start;
        //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
        ////如果起始点为空，设置global_pose为起始点
        if (req.start.header.frame_id.empty())
        {
            geometry_msgs::PoseStamped global_pose;
            if (!getRobotPose(global_pose, planner_costmap_ros_))
            {
                ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
                return false;
            }
            start = global_pose;
        }
        else
        {
            start = req.start;
        }

        if (make_plan_clear_costmap_)
        {
            //update the copy of the costmap the planner uses
            clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
        }

        //first try to make a plan to the exact desired goal
        //制定规划策略：
        std::vector<geometry_msgs::PoseStamped> global_plan;
        if (!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
        {
            ROS_DEBUG_NAMED("move_base", "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                            req.goal.pose.position.x, req.goal.pose.position.y);

            //search outwards for a feasible goal within the specified tolerance
            //  //在规定的公差范围内向外寻找可行的goal
            geometry_msgs::PoseStamped p;
            p = req.goal;
            bool found_legal = false;
            float resolution = planner_costmap_ros_->getCostmap()->getResolution();
            float search_increment = resolution * 3.0;
            if (req.tolerance > 0.0 && req.tolerance < search_increment) ////以3倍分辨率的增量向外寻找
                search_increment = req.tolerance;
            for (float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment)
            {
                for (float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment)
                {
                    for (float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment)
                    {

                        //don't search again inside the current outer layer
                        ////不在本位置的外侧layer查找，太近的不找
                        if (x_offset < max_offset - 1e-9 && y_offset < max_offset - 1e-9)
                            continue;

                        //search to both sides of the desired goal
                        ////从两个方向x、y查找精确的goal
                        for (float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0)
                        {

                            //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
                            if (y_offset < 1e-9 && y_mult < -1.0 + 1e-9)
                                continue;

                            for (float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0)
                            {
                                ////第一次遍历如果偏移量过小,则去除这个点或者上一点
                                if (x_offset < 1e-9 && x_mult < -1.0 + 1e-9)
                                    continue;

                                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                                if (planner_->makePlan(start, p, global_plan))
                                {
                                    if (!global_plan.empty())
                                    {

                                        if (make_plan_add_unreachable_goal_)
                                        {
                                            //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                                            //(the reachable goal should have been added by the global planner)
                                            global_plan.push_back(req.goal);
                                        }

                                        found_legal = true;
                                        ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                        break;
                                    }
                                }
                                else
                                {
                                    ROS_DEBUG_NAMED("move_base", "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                }
                            }
                        }
                    }
                }
            }
        }

        //copy the plan into a message to send out
        //然后把规划后的global_plan附给resp，并且传出去：
        resp.plan.poses.resize(global_plan.size());
        for (unsigned int i = 0; i < global_plan.size(); ++i)
        {
            resp.plan.poses[i] = global_plan[i];
        }

        return true;
    }
    // 2、析构函数
    MoveBase::~MoveBase()
    {
        recovery_behaviors_.clear();

        delete dsrv_;

        if (as_ != NULL)
            delete as_;

        if (planner_costmap_ros_ != NULL)
            delete planner_costmap_ros_;

        if (controller_costmap_ros_ != NULL)
            delete controller_costmap_ros_;

        planner_thread_->interrupt();
        planner_thread_->join();

        delete planner_thread_;

        delete planner_plan_;
        delete latest_plan_;
        delete controller_plan_;

        planner_.reset();
        tc_.reset();
    }
    // 6、新的全局规划，goal 规划的目标点，plan 路径
    bool MoveBase::makePlan(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

        //make sure to set the plan to be empty initially
        plan.clear();

        //since this gets called on handle activate
        if (planner_costmap_ros_ == NULL) //检测代价地图是否传入
        {
            ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
            return false;
        }

        //get the starting pose of the robot
        geometry_msgs::PoseStamped global_pose;
        if (!getRobotPose(global_pose, planner_costmap_ros_)) // //获取机器人的位姿
        {
            ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
            return false;
        }

        const geometry_msgs::PoseStamped &start = global_pose;

        //if the planner fails or returns a zero length plan, planning failed
        //通过智能指针，定义了一个指向nav_core::BaseGlobalPlanner类对象的一个指针，并调用那个类中的成员函数makePlan:
        if (!planner_->makePlan(start, goal, plan) || plan.empty()) //值得注意的是这里的两个makePlan是不同的 global_planner
        {
            ROS_DEBUG_NAMED("move_base", "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
            return false;
        }

        return true;
    }
    // 10、发布速度为0的指令
    void MoveBase::publishZeroVelocity()
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }
    //15、 判断四元素是否有效
    bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion &q)
    {
        //first we need to check if the quaternion has nan's or infs
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
        {
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

        //next, we need to check if the length of the quaternion is close to zero
        if (tf_q.length2() < 1e-6)
        {
            ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
            return false;
        }

        //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        tf_q.normalize();

        tf2::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        if (fabs(dot - 1) > 1e-3)
        {
            ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
            return false;
        }

        return true;
    }
    //16、将目标的坐标系统一转换为全局坐标系：
    geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg)
    {
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        geometry_msgs::PoseStamped goal_pose, global_pose;
        goal_pose = goal_pose_msg;

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.header.stamp = ros::Time();

        try
        {
            tf_.transform(goal_pose_msg, global_pose, global_frame);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                     goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        return global_pose;
    }
    // 12、周期性地唤醒规划器
    void MoveBase::wakePlanner(const ros::TimerEvent &event)
    {
        // we have slept long enough for rate
        planner_cond_.notify_one();
    }
    // 这是planner线程的入口。这个函数需要等待actionlib服务器的cbMoveBase::executeCb来唤醒启动。
    // 主要作用是调用全局路径规划获取路径，同时保证规划的周期性以及规划超时清除goal。
    void MoveBase::planThread()
    {
        ROS_DEBUG_NAMED("move_base_plan_thread", "Starting planner thread...");
        ros::NodeHandle n;
        ros::Timer timer;
        bool wait_for_wake = false;
        ////1. 创建递归锁
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        while (n.ok())
        {
            //check if we should run the planner (the mutex is locked)
            // //2.判断是否阻塞线程
            while (wait_for_wake || !runPlanner_)
            {
                //if we should not be running the planner then suspend this thread
                ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
                //当 std::condition_variable 对象的某个 wait 函数被调用的时候，
                //它使用 std::unique_lock(通过 std::mutex) 来锁住当前线程。
                //当前线程会一直被阻塞，直到另外一个线程在相同的 std::condition_variable 对象上调用了 notification 函数来唤醒当前线程。
                planner_cond_.wait(lock); ////使线程进入睡眠，等待MoveBase::executeCb，以及规划周期的唤醒
                wait_for_wake = false;
            }
            ros::Time start_time = ros::Time::now();

            //time to plan! get a copy of the goal and unlock the mutex
            //  //planner_goal_是在MoveBase::executeCb中得到的目标位姿，需要上锁保证线程安全
            geometry_msgs::PoseStamped temp_goal = planner_goal_;
            lock.unlock(); //解开线程锁。
            ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

            //run planner
            //3. 获取规划的全局路径
            //这里的makePlan作用是获取机器人的位姿作为起点，然后调用全局规划器的makePlan返回规划路径，存储在planner_plan_
            planner_plan_->clear();                                       //清除原来规划出的路径容器
            bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_); //检测代价地图是否传入、获取机器人的位姿。值得注意的是这里的两个makePlan是不同的

            //4.如果获得了plan,则将其赋值给latest_plan_
            if (gotPlan) ////如果规划出路径则更新相应路径，并将state_转换为CONTROLLING状态
            {
                ROS_DEBUG_NAMED("move_base_plan_thread", "Got Plan with %zu points!", planner_plan_->size());
                //pointer swap the plans under mutex (the controller will pull from latest_plan_)
                std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;

                lock.lock();
                planner_plan_ = latest_plan_;
                latest_plan_ = temp_plan; //将最新的全局路径放到latest_plan_中，其在MoveBase::executeCycle中被传递到controller_plan_中，利用锁来进行同步
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                new_global_plan_ = true;

                ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

                //make sure we only start the controller if we still haven't reached the goal
                if (runPlanner_)
                    state_ = CONTROLLING;
                if (planner_frequency_ <= 0)
                    runPlanner_ = false;
                lock.unlock();
            }
            //if we didn't get a plan and we are in the planning state (the robot isn't moving)
            //5.如果没有规划出路径，并且处于PLANNING状态，则判断是否超过最大规划周期或者规划次数
            //如果是则进入自转模式，否则应该会等待MoveBase::executeCycle的唤醒再次规划
            else if (state_ == PLANNING) //仅在MoveBase::executeCb及其调用的MoveBase::executeCycle或者重置状态时会被设置为PLANNING，
            {                            //一般是刚获得新目标，或者得到路径但计算不出下一步控制时重新进行路径规划
                ROS_DEBUG_NAMED("move_base_plan_thread", "No Plan...");
                ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);
                // （2）
                //check if we've tried to make a plan for over our time limit or our maximum number of retries
                //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
                //is negative (the default), it is just ignored and we have the same behavior as ever
                lock.lock();
                planning_retries_++;
                if (runPlanner_ &&
                    (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_)))
                {
                    //we'll move into our obstacle clearing mode
                    state_ = CLEARING;
                    runPlanner_ = false;   // proper solution for issue #523
                    publishZeroVelocity(); //直接向cmd_vel话题发布000的速度信息
                    recovery_trigger_ = PLANNING_R;
                }

                lock.unlock();
            }

            //take the mutex for the next iteration
            lock.lock();

            //setup sleep interface if needed
            //6.设置睡眠模式
            ////如果还没到规划周期则定时器睡眠，在定时器中断中通过planner_cond_唤醒，这里规划周期为0
            //其中，planner_cond_专门用于开启路径规划的线程，在其他地方也经常遇到；
            //这一步中，实现了从latest_plan_到planner_plan_ 的跨越；MoveBase::wakePlanner（）函数中用planner_cond_开启了路径规划的线程。
            if (planner_frequency_ > 0)
            {
                ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_frequency_)) - ros::Time::now();
                if (sleep_time > ros::Duration(0.0))
                {
                    wait_for_wake = true;
                    timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
                }
            }
        }
    }
    // 14 、as_  action 的客户端 actionlib 服务的回调函数 action 的客户端触发回调线程
    void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal)
    {
        /*这个线程主要用于接收目标点，并进行坐标变换，并转化为成员变量，将其用话题的方式再发布出去。
         boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_); boost::recursive_mutex planner_mutex_;
            给该线程上锁。并进行赋值操作，赋值完毕后，用lock.unlock()打开互斥锁。并往”current_goal“这个话题发送消息。*/

        //该函数流程是
        //1. 第一次进入后接收goal，判断有效性等，然后开启规划线程得到路径。
        //2. 然后在while(n.ok())循环中调用executeCycle(goal, global_plan);来控制机器人进行相应跟随。
        //3. 期间会不断检测是否有新的goal抢占，或者坐标系变换等，如果有则在while循环中重复步骤1的初始化再跟随
        //4. 如果有被空抢占（如cancel等）则清除退出
        //5. 如果跟随完成则退出
        //6. 会进行控制周期约束。

        //判断goal有效性
        if (!isQuaternionValid(move_base_goal->target_pose.pose.orientation)) //并进行坐标变换，并转化为成员变量,用isQuaternionValid判断发出的目标点的有效性
        {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
        }
        //统一转换到全局坐标系
        geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose); //传入目标点的位姿,返回goal相对于Global坐标系的坐标。

        publishZeroVelocity(); //用于初始化发送都为0的cmd信号
        //we have a goal so start the planner 设置目标点并唤醒路径规划线程：
        //当一个线程被Lock,其他线程只能等待.
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_); //给该线程上锁 unique_lock是独占的 boost::recursive_mutex是重入锁或者称为递归锁
        planner_goal_ = goal;
        // //启动所说的新线程来获取规划路径
        runPlanner_ = true;
        //唤醒等待条件变量的一个线程：即调用planner_cond_.wait()的MoveBase::planThread()
        planner_cond_.notify_one(); ////Notify one waiting thread, if there is one.
        lock.unlock();              //打开互斥锁

        // 然后发布goal，设置控制频率
        current_goal_pub_.publish(goal); //把goal发布给可视化工具
        std::vector<geometry_msgs::PoseStamped> global_plan;

        // 开启costmap更新：
        ros::Rate r(controller_frequency_);
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Starting up costmaps that were shut down previously");
            planner_costmap_ros_->start();
            controller_costmap_ros_->start();
        }

        //we want to make sure that we reset the last time we had a valid plan and control
        // 重置时间标志位：
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;

        ros::NodeHandle n;
        // 开启循环，循环判断是否有新的goal抢占（重要！！！）：
        while (n.ok())
        {
            //  7. 修改循环频率
            if (c_freq_change_)
            {
                ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
                r = ros::Rate(controller_frequency_);
                c_freq_change_ = false;
            }
            //8. 如果获得一个抢占式目标
            // //是否有抢占请求，根据参考1第8点的说法，SimpleActionServer的政策是，新的goal都会抢占旧的goal，
            // 这里应该只是为了清除新goal的一些状态。（那些待定的goal也有可能抢占，或者可以直接cancel抢占Current？）
            if (as_->isPreemptRequested()) //action的抢断函数
            {
                if (as_->isNewGoalAvailable()) ///如果是新的goal这个函数会将其他goal设置为被抢占状态
                {
                    //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                    //如果有新的目标，会接受的，但不会关闭其他进程
                    move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal(); ///接收new goal
                    //9.如果目标无效,则返回
                    if (!isQuaternionValid(new_goal.target_pose.pose.orientation))
                    {
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
                        return; ////无效退出
                    }
                    //10.将目标转换为全局坐标系
                    goal = goalToGlobalFrame(new_goal.target_pose); //统一转换到全局坐标系

                    //we'll make sure that we reset our state for the next execution cycle
                    //11.设置状态为PLANNING
                    recovery_index_ = 0;
                    state_ = PLANNING;

                    //we have a new goal so make sure the planner is awake
                    //12. 设置目标点并唤醒路径规划线程
                    lock.lock(); //唤醒规划线程
                    planner_goal_ = goal;
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                    lock.unlock();

                    //publish the goal point to the visualizer
                    //13. 把goal发布给可视化工具
                    ROS_DEBUG_NAMED("move_base", "move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
                    current_goal_pub_.publish(goal);

                    //make sure to reset our timeouts and counters
                    //14. 重置规划时间
                    last_valid_control_ = ros::Time::now();
                    last_valid_plan_ = ros::Time::now();
                    last_oscillation_reset_ = ros::Time::now();
                    planning_retries_ = 0;
                }
                else ////如果是cancel了
                {
                    //if we've been preempted explicitly we need to shut things down
                    //14.重置状态,设置为抢占式任务
                    resetState(); //停止规划线程、停车等

                    //notify the ActionServer that we've successfully preempted
                    //通知ActionServer已成功抢占
                    ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");
                    as_->setPreempted(); ////设置current goal被抢占

                    //we'll actually return from execute after preempting
                    return;
                }
            }

            //we also want to check if we've changed global frames because we need to transform our goal pose
            //15.如果目标点的坐标系和全局地图的坐标系不相同
            if (goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()) ////判断这段时间是否改了坐标系
            {
                //16,转换目标点坐标系
                goal = goalToGlobalFrame(goal);

                //we want to go back to the planning state for the next execution cycle
                recovery_index_ = 0;
                state_ = PLANNING;

                //we have a new goal so make sure the planner is awake
                //17. 设置目标点并唤醒路径规划线程
                lock.lock();
                planner_goal_ = goal;
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();

                //publish the goal point to the visualizer
                ROS_DEBUG_NAMED("move_base", "The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
                current_goal_pub_.publish(goal);

                //make sure to reset our timeouts and counters
                //18.重置规划器相关时间标志位
                last_valid_control_ = ros::Time::now();
                last_valid_plan_ = ros::Time::now();
                last_oscillation_reset_ = ros::Time::now();
                planning_retries_ = 0;
            }

            //for timing that gives real time even in simulation
            ros::WallTime start = ros::WallTime::now();

            //the real work on pursuing a goal is done here
            //19. 到达目标点的真正工作，控制机器人进行跟随
            bool done = executeCycle(goal, global_plan); //这是控制机器人跟踪的主要函数，

            //if we're done, then we'll return from execute
            //20.如果完成任务则返回
            //  其中，done的值即为MoveBase::executeCycle（）函数的返回值，
            //  这个值非常重要，直接判断了是否到达目标点；MoveBase::executeCycle（）函数是控制机器人进行跟随的函数（重要！！！），
            if (done)
                return;

            //check if execution of the goal has completed in some way
            ros::WallDuration t_diff = ros::WallTime::now() - start;
            ROS_DEBUG_NAMED("move_base", "Full control cycle time: %.9f\n", t_diff.toSec());

            //21. 执行休眠动作
            r.sleep(); //控制周期睡眠
            //make sure to sleep for the remainder of our cycle time
            if (r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
                ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
        }

        //wake up the planner thread so that it can exit cleanly
        //22. 唤醒计划线程，以便它可以干净地退出
        lock.lock();
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //if the node is killed then we'll abort and return/
        //23. 如果节点结束就终止并返回
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
        return;
    }
    //16、距离计算函数d= sqrt(dx^2 + dy^2)
    double MoveBase::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
    {
        return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    }
    // 3、 控制闭环、全局规划、 到达目标返回true，没有到达返回false
    bool MoveBase::executeCycle(geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &global_plan)
    {
        // 该函数的两个参数分别是目标点位姿以及规划出的全局路径。
        // 实现的是通过上述两个已知，利用局部路径规划器直接输出轮子速度，控制机器人按照路径走到目标点，成功返回真，否则返回假。
        // 在actionlib server的回调MoveBase::executeCb中被调用。
        boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        //we need to be able to publish velocity commands
        geometry_msgs::Twist cmd_vel;

        //update feedback to correspond to our curent position
        geometry_msgs::PoseStamped global_pose;
        getRobotPose(global_pose, planner_costmap_ros_);
        const geometry_msgs::PoseStamped &current_position = global_pose;

        //push the feedback out
        ///变量定义并获取机器人坐标发布给server的feedback
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_->publishFeedback(feedback);

        //check to see if we've moved far enough to reset our oscillation timeout
        // 判断当前位置和是否振荡，其中distance函数返回的是两个位置的直线距离（欧氏距离），recovery_trigger_是枚举RecoveryTrigger的对象：
        if (distance(current_position, oscillation_pose_) >= oscillation_distance_)
        {
            last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position;

            //if our last recovery was caused by oscillation, we want to reset the recovery index
            //如果上次的恢复是由振荡引起的，重置恢复指数
            if (recovery_trigger_ == OSCILLATION_R)
                recovery_index_ = 0;
        }

        //check that the observation buffers for the costmap are current, we don't want to drive blind
        if (!controller_costmap_ros_->isCurrent()) // //如果观测传感器数据不更新，则让机器人停机并退出函数 停止机器人，返回false
        {
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety", ros::this_node::getName().c_str());
            publishZeroVelocity(); //把geometry_msgs::Twist类型的cmd_vel设置为0并发布出去:
            return false;
        }

        //if we have a new plan then grab it and give it to the controller
        ////该变量在规划器线程中，当新的路径被规划出来，该值被置1  //完成latest_plan_到controller_plan_的转换 （提示：头文件那里讲过规划转换的规则）：
        if (new_global_plan_)
        {
            //make sure to set the new plan flag to false
            new_global_plan_ = false;

            ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

            //do a pointer swap under mutex
            std::vector<geometry_msgs::PoseStamped> *temp_plan = controller_plan_;

            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            controller_plan_ = latest_plan_;
            latest_plan_ = temp_plan;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base", "pointers swapped!");

            ////将全局路径设置到局部路径规划器中 //5. 给控制器设置全局路径
            //其中，tc_是局部规划器的指针，setPlan是TrajectoryPlannerROS的函数（注意！！！跟外部包有关系了！！）
            if (!tc_->setPlan(*controller_plan_))
            {
                //ABORT and SHUTDOWN COSTMAPS //同时也关闭规划器线程，没必要规划了
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();

                //disable the planner thread
                lock.lock();
                runPlanner_ = false;
                lock.unlock();
                //6.设置动作中断,返回true
                as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
                return true;
            }

            //make sure to reset recovery_index_ since we were able to find a valid plan
            ////如果全局路径有效，则不需要recovery
            if (recovery_trigger_ == PLANNING_R)
                recovery_index_ = 0;
        }

        //the move_base state machine, handles the control logic for navigation
        ////对状态机进行处理
        /*然后判断move_base状态，一般默认状态或者接收到一个有效goal时是PLANNING，
        在规划出全局路径后state_会由PLANNING变为CONTROLLING，
        如果规划失败则由PLANNING变为CLEARING，架构如下：
        switch(state_){
            case PLANNING:
            case CONTROLLING:
            case CLEARING:
            default:
             } */
        switch (state_)
        {
        //if we are in a planning state, then we'll attempt to make a plan
        case PLANNING: //唤醒规划线程 机器人规划状态，尝试获取一条全局路径
        {
            boost::recursive_mutex::scoped_lock lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one(); ////还在PLANNING中则唤醒规划线程让它干活
        }
            ROS_DEBUG_NAMED("move_base", "Waiting for plan, in the planning state.");
            break;

        //if we're controlling, we'll attempt to find valid velocity commands
        case CONTROLLING: //机器人控制状态，尝试获取一个有效的速度命令：
            ROS_DEBUG_NAMED("move_base", "In controlling state.");

            //check to see if we've reached our goal
            if (tc_->isGoalReached()) //如果到达目标点，重置状态，设置动作成功，返回true
            {
                ROS_DEBUG_NAMED("move_base", "Goal reached!");
                resetState(); //其中，resetState（）函数如下，配合上面的看，机器人到达目标点后把move_base状态设置为PLANNING：

                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();

                // //重置状态，关闭规划器线程，设置告知Client结果
                as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
                return true;
            }
            //以下三种情况，开始clearing
            //check for an oscillation condition （1）如果超过震荡时间，停止机器人，设置清障标志位：
            //last_oscillation_reset_获得新目标会重置,距离超过震荡距离（默认0.5）会重置，进行recovery后会重置
            //所以是太久没有发生上面的事就震动一下，防止长时间在同一个地方徘徊？？？？这里oscillation_timeout_默认为0 ，不发生。
            if (oscillation_timeout_ > 0.0 &&
                last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
            {
                publishZeroVelocity();
                state_ = CLEARING;
                recovery_trigger_ = OSCILLATION_R;
            }

            {
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

                if (tc_->computeVelocityCommands(cmd_vel)) //如果局部路径规划成功 获取有效速度,如果获取成功，直接发布到cmd_vel：
                {
                    ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
                    last_valid_control_ = ros::Time::now();
                    //make sure that we send the velocity command to the base
                    vel_pub_.publish(cmd_vel); //发布控制速度信息
                    if (recovery_trigger_ == CONTROLLING_R)
                        recovery_index_ = 0;
                }
                else //局部规划失败 没有获取有效速度
                {
                    ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                    ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                    //check if we've tried to find a valid control for longer than our time limit （3）
                    if (ros::Time::now() > attempt_end) //判断是否超过尝试时间，如果超时，则停止机器人，进入清障模式
                    {
                        //we'll move into our obstacle clearing mode
                        publishZeroVelocity();
                        state_ = CLEARING;
                        recovery_trigger_ = CONTROLLING_R;
                    }
                    else ////没超时则启动规划器线程重新规划 如果没有超时，则再全局规划一个新的路径：
                    {
                        //otherwise, if we can't find a valid control, we'll go back to planning
                        last_valid_plan_ = ros::Time::now();
                        planning_retries_ = 0;
                        state_ = PLANNING;
                        publishZeroVelocity();

                        //enable the planner thread in case it isn't running on a clock
                        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                        runPlanner_ = true;
                        planner_cond_.notify_one();
                        lock.unlock();
                    }
                }
            }

            break;

        //we'll try to clear out space with any user-provided recovery behaviors
        case CLEARING: //三种原因需要recovery，上面有说 机器人清障状态，规划或者控制失败，恢复或者进入到清障模式：
            ROS_DEBUG_NAMED("move_base", "In clearing/recovery state");
            //we'll invoke whatever recovery behavior we're currently on if they're enabled
            if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()) //遍历recovery方法 如果有可用恢复器，执行恢复动作，并设置状态为PLANNING：
            {
                ROS_DEBUG_NAMED("move_base_recovery", "Executing behavior %u of %zu", recovery_index_ + 1, recovery_behaviors_.size());

                move_base_msgs::RecoveryStatus msg;
                msg.pose_stamped = current_position;
                msg.current_recovery_number = recovery_index_;
                msg.total_number_of_recoveries = recovery_behaviors_.size();
                msg.recovery_behavior_name = recovery_behavior_names_[recovery_index_];

                recovery_status_pub_.publish(msg);
                //其中，recovery_behaviors_类型为std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> >，
                //runBehavior（）函数为move_slow_and_clear包里面的函数。（注意！！！链接到外部包！！！）。
                recovery_behaviors_[recovery_index_]->runBehavior(); //

                //we at least want to give the robot some time to stop oscillating after executing the behavior
                last_oscillation_reset_ = ros::Time::now();

                //we'll check if the recovery behavior actually worked
                ROS_DEBUG_NAMED("move_base_recovery", "Going back to planning state");
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                state_ = PLANNING;

                //update the index of the next recovery behavior that we'll try
                recovery_index_++;
            }
            else //遍历完还是不行 如果没有可用恢复器，结束动作，返回true：
            {
                ROS_DEBUG_NAMED("move_base_recovery", "All recovery behaviors have failed, locking the planner and disabling it.");
                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();

                ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");
                // 三种恢复机制:
                if (recovery_trigger_ == CONTROLLING_R) //终止方式 1 //分原因发布消息
                {
                    ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
                }
                else if (recovery_trigger_ == PLANNING_R) //终止方式 2
                {
                    ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
                }
                else if (recovery_trigger_ == OSCILLATION_R) //终止方式 3
                {
                    ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
                }
                resetState();
                return true;
            }
            break;
            //（4）除了上述状态以外：
        default:
            ROS_ERROR("This case should never be reached, something is wrong, aborting");
            resetState();
            //disable the planner thread
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = false;
            lock.unlock();
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
            return true; //已经done了。
        }

        //we aren't done yet
        return false;
    }
    // 7、从参数服务器加载导航恢复行为
    bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
    {
        XmlRpc::XmlRpcValue behavior_list;
        if (node.getParam("recovery_behaviors", behavior_list))
        {
            if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for (int i = 0; i < behavior_list.size(); ++i)
                {
                    if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                    {
                        if (behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type"))
                        {
                            //check for recovery behaviors with the same name
                            for (int j = i + 1; j < behavior_list.size(); j++)
                            {
                                if (behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                                {
                                    if (behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type"))
                                    {
                                        std::string name_i = behavior_list[i]["name"];
                                        std::string name_j = behavior_list[j]["name"];
                                        if (name_i == name_j)
                                        {
                                            ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                                      name_i.c_str());
                                            return false;
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                            return false;
                        }
                    }
                    else
                    {
                        ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                                  behavior_list[i].getType());
                        return false;
                    }
                }

                //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
                for (int i = 0; i < behavior_list.size(); ++i)
                {
                    try
                    {
                        //check if a non fully qualified name has potentially been passed in
                        if (!recovery_loader_.isClassAvailable(behavior_list[i]["type"]))
                        {
                            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                            for (unsigned int i = 0; i < classes.size(); ++i)
                            {
                                if (behavior_list[i]["type"] == recovery_loader_.getName(classes[i]))
                                {
                                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                                    ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                             std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                    behavior_list[i]["type"] = classes[i];
                                    break;
                                }
                            }
                        }

                        boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

                        //shouldn't be possible, but it won't hurt to check
                        if (behavior.get() == NULL)
                        {
                            ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                            return false;
                        }

                        //initialize the recovery behavior with its name
                        behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
                        recovery_behavior_names_.push_back(behavior_list[i]["name"]);
                        recovery_behaviors_.push_back(behavior);
                    }
                    catch (pluginlib::PluginlibException &ex)
                    {
                        ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                        return false;
                    }
                }
            }
            else
            {
                ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                          behavior_list.getType());
                return false;
            }
        }
        else
        {
            //if no recovery_behaviors are specified, we'll just load the defaults
            return false;
        }

        //if we've made it here... we've constructed a recovery behavior list successfully
        return true;
    }
    // 8、加载默认导航恢复行为 //we'll load our default recovery behaviors here
    void MoveBase::loadDefaultRecoveryBehaviors()
    {
        recovery_behaviors_.clear();
        try
        {
            //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
            ros::NodeHandle n("~");
            n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
            n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

            //first, we'll load a recovery behavior to clear the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("conservative_reset");
            recovery_behaviors_.push_back(cons_clear);

            //next, we'll load a recovery behavior to rotate in place
            boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
            if (clearing_rotation_allowed_)
            {
                rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
                recovery_behavior_names_.push_back("rotate_recovery");
                recovery_behaviors_.push_back(rotate);
            }

            //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("aggressive_reset");
            recovery_behaviors_.push_back(ags_clear);

            //we'll rotate in-place one more time
            if (clearing_rotation_allowed_)
            {
                recovery_behaviors_.push_back(rotate);
                recovery_behavior_names_.push_back("rotate_recovery");
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
        }

        return;
    }
    // 11、重置move_base action的状态，设置速度为0
    void MoveBase::resetState()
    {
        // Disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        // Reset statemachine
        state_ = PLANNING;
        recovery_index_ = 0;
        recovery_trigger_ = PLANNING_R;
        publishZeroVelocity();

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
    }
    //17、获取位姿信息函数
    bool MoveBase::getRobotPose(geometry_msgs::PoseStamped &global_pose, costmap_2d::Costmap2DROS *costmap)
    {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = ros::Time();     // latest available
        ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

        // get robot pose on the given costmap frame
        // 转换到统一的全局坐标
        try
        {
            tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        }
        catch (tf2::LookupException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        // check if global_pose time stamp is within costmap transform tolerance
        // 全局坐标时间戳是否在costmap要求下
        if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
        {
            ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. "
                                   "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f",
                              costmap->getName().c_str(),
                              current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
            return false;
        }

        return true;
    }
};
