#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <arm_message/srv/place.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

int e=1;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(const geometry_msgs::msg::PoseStamped& pose);

private:

  void handlePlace(
    const std::shared_ptr<arm_message::srv::Place::Request> request,
    std::shared_ptr<arm_message::srv::Place::Response> response);
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(const geometry_msgs::msg::PoseStamped& pose);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<arm_message::srv::Place>::SharedPtr place_service_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  place_service_ = node_->create_service<arm_message::srv::Place>(
  "place",
  std::bind(
    &MTCTaskNode::handlePlace,
    this,
    std::placeholders::_1,
    std::placeholders::_2));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}


void MTCTaskNode::handlePlace(
  const std::shared_ptr<arm_message::srv::Place::Request> request,
  std::shared_ptr<arm_message::srv::Place::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Place request received");

  doTask(request->pose);

  response->success = (e == 0);
}

void MTCTaskNode::doTask(const geometry_msgs::msg::PoseStamped& pose)
{

  auto gripper_group =
  std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, "hand");
  moveit::planning_interface::MoveGroupInterface::Plan grip_plan;
  gripper_group->setNamedTarget("close");

  bool grip_plan_success =
      (gripper_group->plan(grip_plan) ==
      moveit::core::MoveItErrorCode::SUCCESS);

  if (!grip_plan_success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Gripper close planning failed");
    return;
  }

  auto grip_exec_result = gripper_group->execute(grip_plan);

  rclcpp::sleep_for(std::chrono::seconds(5));

  task_ = createTask(pose);

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(2 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
  task_.reset();
  task_.clear();

  e=0;

  return;
}

mtc::Task MTCTaskNode::createTask(const geometry_msgs::msg::PoseStamped& pose)
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm_hand";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "grasp_frame";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");
  sampling_planner->setTimeout(15.0);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // clang-format off
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg=pose;
      // target_pose_msg.header.frame_id = "world";
      //   target_pose_msg.pose.position.y = 0.0;
      //   target_pose_msg.pose.position.x = 0.5;
        target_pose_msg.pose.orientation.y = 0.7071;
        target_pose_msg.pose.orientation.w = 0.7071;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(current_state_ptr);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  auto home =
      std::make_unique<mtc::stages::MoveTo>("home", sampling_planner);
  home->setGroup(arm_group_name);
  home->setGoal("cam");
  task.add(std::move(home));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}