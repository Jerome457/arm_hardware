#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <cmath> 
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
#include <arm_message/srv/pick.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;
int e=1;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

private:
  void handlePick(
    const std::shared_ptr<arm_message::srv::Pick::Request> request,
    std::shared_ptr<arm_message::srv::Pick::Response> response);

  void setupPlanningScene(
    const std::string& object_id,
    const geometry_msgs::msg::PoseStamped& pose);

  void doTask();

  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Service<arm_message::srv::Pick>::SharedPtr pick_service_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("pick_and_place_server", options) }
{
  pick_service_ = node_->create_service<arm_message::srv::Pick>(
    "Pick_and_Place",
    std::bind(
      &MTCTaskNode::handlePick,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(node_->get_logger(), "Pick_and_Place service ready");
}


void MTCTaskNode::setupPlanningScene(
  const std::string& object_id,
  const geometry_msgs::msg::PoseStamped& pose)

{
  // ----- Add Mesh Object -----
  moveit_msgs::msg::CollisionObject mesh_obj;
  mesh_obj.id = object_id;
  mesh_obj.header.frame_id = pose.header.frame_id;


  // Load the mesh resource (from your package)
  std::string mesh_path = "package://arm_urdf/rcup_objects/M30-1.stl";

  // Scale the mesh down (0.001 = 1/1000th original size)
  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path, Eigen::Vector3d(0.0001, 0.0001, 0.0001));
  if (!mesh)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load mesh from: %s", mesh_path.c_str());
    return;
  }

  // Convert shape to message
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);
  shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  // Set mesh pose (control position + orientation)
  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose = pose.pose;

  // Example: Rotate 90° about Y-axis, 45° about Z-axis
  tf2::Quaternion q;
  q.setRPY(0.0,M_PI/2, 0.0);  // (roll, pitch, yaw)
  mesh_pose.orientation = tf2::toMsg(q);

  // Fill collision object fields
  mesh_obj.meshes.push_back(mesh_msg);
  mesh_obj.mesh_poses.push_back(mesh_pose);
  mesh_obj.operation = mesh_obj.ADD;

  // ----- Add Ground Plane -----
  moveit_msgs::msg::CollisionObject ground;
  ground.id = "ground_plane";
  ground.header.frame_id = "world";

  shape_msgs::msg::SolidPrimitive ground_shape;
  ground_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  ground_shape.dimensions = {4.0, 4.0, 0.01};  // large thin box

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.position.z = -0.15;  // top of ground at z=0
  ground_pose.orientation.w = 1.0;

  ground.primitives.push_back(ground_shape);
  ground.primitive_poses.push_back(ground_pose);
  ground.operation = ground.ADD;

  // ----- Add Colors -----
  moveit_msgs::msg::ObjectColor mesh_color;
  mesh_color.id = "object";
  mesh_color.color.r = 0.0;
  mesh_color.color.g = 1.0;
  mesh_color.color.b = 0.0;
  mesh_color.color.a = 1.0;

  moveit_msgs::msg::ObjectColor ground_color;
  ground_color.id = "ground_plane";
  ground_color.color.r = 0.0;
  ground_color.color.g = 0.0;
  ground_color.color.b = 1.0;
  ground_color.color.a = 0.0;

  // ----- Publish Planning Scene -----
  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true;
  scene_msg.world.collision_objects.push_back(mesh_obj);
  scene_msg.world.collision_objects.push_back(ground);
  scene_msg.object_colors.push_back(mesh_color);
  scene_msg.object_colors.push_back(ground_color);

  // Publisher must stay alive briefly to ensure the message is sent
  auto planning_scene_pub =
      node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

  rclcpp::Rate rate(10);
  for (int i = 0; i < 5; ++i)
  {
    planning_scene_pub->publish(scene_msg);
    rate.sleep();
  }

}

void MTCTaskNode::handlePick(
  const std::shared_ptr<arm_message::srv::Pick::Request> request,
  std::shared_ptr<arm_message::srv::Pick::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Pick request received for object: %s",
              request->object_id.c_str());

  setupPlanningScene(request->object_id, request->pose);

  doTask();

  response->success = (e == 0);
}


void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(2))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    e=1;
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    e=1;
    return;
  }
  
auto move_group =
  std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, "arm_hand");

auto gripper_group =
  std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, "hand");

  move_group->setPlannerId("TRRTkConfigDefault");

  moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
  moveit::planning_interface::MoveGroupInterface::Plan grip_plan;

  gripper_group->setNamedTarget("close");
  rclcpp::sleep_for(std::chrono::seconds(5));

  bool grip_plan_success =
      (gripper_group->plan(grip_plan) ==
      moveit::core::MoveItErrorCode::SUCCESS);

  if (!grip_plan_success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Gripper close planning failed");
    return;
  }

  auto grip_exec_result = gripper_group->execute(grip_plan);

  if (grip_exec_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Gripper close execution failed");
    return;
  }

  move_group->setNamedTarget("place");

  bool arm_plan_success =
      (move_group->plan(arm_plan) ==
      moveit::core::MoveItErrorCode::SUCCESS);

  if (!arm_plan_success)
  {
    RCLCPP_ERROR(node_->get_logger(), "Planning to place failed");
    return;
  }

  auto arm_exec_result = move_group->execute(arm_plan);

  if (arm_exec_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Execution to place failed");
    return;
  }

  gripper_group->setNamedTarget("open");

  bool grip_plan_success2 =
      (gripper_group->plan(grip_plan) ==
      moveit::core::MoveItErrorCode::SUCCESS);

  if (!grip_plan_success2)
  {
    RCLCPP_ERROR(node_->get_logger(), "Gripper open planning failed");
    return;
  }

  rclcpp::sleep_for(std::chrono::seconds(5));

  auto grip_exec_result2 = gripper_group->execute(grip_plan);

  if (grip_exec_result2 != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Gripper open execution failed");
    return;
  }

  move_group->setNamedTarget("cam");

  // bool arm_plan_success2 =
  //     (move_group->plan(arm_plan) ==
  //     moveit::core::MoveItErrorCode::SUCCESS);

  // if (!arm_plan_success2)
  // {
  //   RCLCPP_ERROR(node_->get_logger(), "Planning to cam failed");
  //   return;
  // }

  // auto arm_exec_result2 = move_group->execute(arm_plan);

  // if (arm_exec_result2 != moveit::core::MoveItErrorCode::SUCCESS)
  // {
  //   RCLCPP_ERROR(node_->get_logger(), "Move to cam failed");
  //   return;
  // }

  RCLCPP_INFO(node_->get_logger(), "Full sequence completed successfully");

  e=0;
  return;
}

mtc::Task MTCTaskNode::createTask()
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
  

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;
#pragma GCC diagnostic pop
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  sampling_planner->setPlannerId("TRRTkConfigDefault");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.05);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
stage_move_to_pick->setTimeout(100.0);
stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
task.add(std::move(stage_move_to_pick));
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
      grasp->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });

      {
        auto stage =
            std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
        stage->properties().set("marker_ns", "approach_object");
        stage->properties().set("link", hand_frame);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.0, 0.15);
      
        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.z = -0.05;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      {
        // Sample grasp pose
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose("open");
        stage->setObject("object");
        stage->setAngleDelta(M_PI / 18);
        stage->setMonitoredStage(current_state_ptr);  // Hook into current state

        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();
        grasp_frame_transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
        // Compute IK
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(10);
        wrapper->setMinSolutionDistance(0.1);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));
      }

      {
        auto stage =
            std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (soft_fingers,object)");
        stage->allowCollisions("object", "right_H-v1", true);
        stage->allowCollisions("object", "left_H-v1", true);
        // stage->allowCollisions("object", "firstDOF_final", true);
        // stage->allowCollisions("firstDOF_final", "Soft_finger_1", true);
        // stage->allowCollisions("firstDOF_final", "soft_finger_2", true);
        stage->allowCollisions("object", "<octomap>", true);
        grasp->insert(std::move(stage));
      }

      {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
      }
      task.add(std::move(grasp));
    }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_task_node->getNodeBaseInterface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}