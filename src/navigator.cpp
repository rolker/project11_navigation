#include <project11_navigation/navigator.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <project11_navigation/context.h>
#include <project11_navigation/utilities.h>
#include <project11_navigation/task_list.h>
#include <filesystem>
#include <project11_navigation/bt_types.h>

namespace project11_navigation
{

Navigator::Navigator(std::string name):
  action_server_(node_handle_, name+"/run_tasks", false)
{
  registerJsonDefinitions();
  
  ros::NodeHandle nh;
  odom_sub_ = nh.subscribe("odom", 10, &Navigator::odometryCallback, this);

  ros::NodeHandle private_nh("~");

  context_ = std::make_shared<Context>();

  display_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);

  blackboard_ = BT::Blackboard::create();

  blackboard_->set("context", context_);

  tree_ = buildBehaviorTree();
  
  BT::printTreeRecursively(tree_.rootNode());

  groot_ = std::make_shared<BT::Groot2Publisher>(tree_);

  std::string log_file = private_nh.param("log_file", std::string()); 

  if(!log_file.empty())
    logger_ = std::make_shared<BT::FileLogger2>(tree_, log_file);

  bool debug = private_nh.param<bool>("debug",false);
  if(debug)
    console_logger_ = std::make_shared<BT::StdCoutLogger>(tree_);

  action_server_.registerGoalCallback(std::bind(&Navigator::goalCallback, this));
  action_server_.registerPreemptCallback(std::bind(&Navigator::preemptCallback, this));
  action_server_.start();


  //iterate_timer_ = node_handle_.createTimer(ros::Duration(controller_period), &Navigator::iterate, this);
}

Navigator::~Navigator()
{

}

BT::Tree Navigator::buildBehaviorTree()
{
  BT::BehaviorTreeFactory factory;

  factory.registerFromROSPlugins();

  /// \todo Allow extra directories from param server

  std::vector<std::string> packages = {"project11_navigation"};

  for (auto package: packages)
  {
    std::string tree_dir = ros::package::getPath(package)+"/behavior_trees";
    for (auto const& entry : std::filesystem::directory_iterator(tree_dir)) 
    {
      if( entry.path().extension() == ".xml")
      {
        factory.registerBehaviorTreeFromFile(entry.path().string());
      }
    }
  }

  return factory.createTree("NavigatorSequence", blackboard_);
}

void Navigator::goalCallback()
{
  auto goal = action_server_.acceptNewGoal();
  ROS_DEBUG_STREAM("update with " << goal->tasks.size() << " tasks");

  tree_.haltTree();

  if(goal->tasks.empty())
    blackboard_->set("task_messages", std::shared_ptr<std::vector<project11_nav_msgs::TaskInformation> >());
  else
    blackboard_->set("task_messages", std::make_shared<std::vector<project11_nav_msgs::TaskInformation> >(goal->tasks));
}

void Navigator::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  context_->robot().odometryCallback(msg);

  if(action_server_.isActive())
  {
    auto status = tree_.tickOnce();
    switch(status)
    {
      case BT::NodeStatus::SUCCESS:
      {
        project11_navigation::RunTasksResult result;
        auto task_list_entry = blackboard_->getEntry("task_list");
        if(task_list_entry)
        {
          auto task_list = task_list_entry->value.cast<std::shared_ptr<TaskList> >();
          if(task_list)
            result.tasks = task_list->taskMessages();
        }
        action_server_.setSucceeded(result);
        break;
      }
      case BT::NodeStatus::FAILURE:
        action_server_.setAborted();
        break;
      case BT::NodeStatus::RUNNING:
      {
        auto cmd_vel = blackboard_->getEntry("command_velocity");
        if(cmd_vel && cmd_vel->value.isType<geometry_msgs::TwistStamped>())
        {
          context_->robot().sendControls(cmd_vel->value.cast<geometry_msgs::TwistStamped>());
        }

        auto marker_array = blackboard_->getEntry("marker_array");
        if(marker_array && marker_array->value.isType<std::shared_ptr<visualization_msgs::MarkerArray> >())
        {
          auto ma = marker_array->value.cast<std::shared_ptr<visualization_msgs::MarkerArray> >();
          if(ma)
            display_pub_.publish(*ma);
        }
      }
      break;
    }

    project11_navigation::RunTasksFeedback feedback;
    auto current_nav_task = blackboard_->getEntry("active_task_id");
    if(current_nav_task && current_nav_task->value.isString())
      feedback.feedback.current_navigation_task = current_nav_task->value.cast<std::string>();
    auto task_list_entry = blackboard_->getEntry("task_list");
    if(task_list_entry)
    {
      auto task_list = task_list_entry->value.cast<std::shared_ptr<TaskList> >();
      if(task_list)
        feedback.feedback.tasks = task_list->taskMessages();
    }
    action_server_.publishFeedback(feedback);
  }

}

void Navigator::preemptCallback()
{
  action_server_.setPreempted();
  blackboard_->set("task_messages", std::shared_ptr<std::vector<project11_nav_msgs::TaskInformation> >());
  tree_.haltTree();
}


} // namespace project11_navigation
