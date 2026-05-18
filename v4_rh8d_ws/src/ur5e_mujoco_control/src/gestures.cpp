/*
                                        Gestures Demo: UR5e and RH8D Hand
This node demonstrates using MoveIt to plan and execute a sequence of arm poses, while also calling a custom hand gesture service to change the robot's hand configuration.
Flow of the demo:
1. The arm moves to a "Gestures" pose.
2. The hand executes a series of gestures: fully open, fully closed, index point, middle point, victory sign, and then fully open again.
3. Finally, the arm returns to the home pose.
*/

#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <rh8d_mujoco_interfaces/srv/hand_command.hpp>

// Bring these MoveIt types into the current scope so we can use them without full namespace
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
using moveit::core::MoveItErrorCode;

// Create a shortcut name 'HandCommand' for the full service type 'rh8d_mujoco_interfaces::srv::HandCommand'
using HandCommand = rh8d_mujoco_interfaces::srv::HandCommand;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gestures");

class UR5eGestureDemo : public rclcpp::Node
{
public:
  UR5eGestureDemo()
  : Node("gestures", // Constructor function (: calls the parent class constructor)
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) // creats temporary NodeOptions object and enable automatic parameters.
  {
        // Create a ROS2 service client which calls service: /rh8d/hand_command (this = pointer to the current object -> = “access member through pointer”)
    hand_client_ = this->create_client<HandCommand>("/rh8d/hand_command");
  }

  // Main execution function for the demo
  void run()
  {
    // static = this variable (PLANNING_GROUP) is shared everywhere in the class, only one copy exists in memory
    static const std::string PLANNING_GROUP = "ur5e_manipulator";

    // Create a smart shared pointer to a MoveGroupInterface object
    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);

    // move_group set parameters
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.10);
    move_group_->setMaxAccelerationScalingFactor(0.10);
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_->getPlanningFrame().c_str()); // Convert C++ string into C-style string. Because %s needs C-style strings
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_->getEndEffectorLink().c_str());

    // Adds environment obstacles to MoveIt:
    add_table_collision();
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto names = move_group_->getNamedTargets(); // auto: C++ automatically figures out the type
    
    // Loop through all target names in 'names' without copying each string (using reference &)
    for (const auto &name : names) {
      RCLCPP_INFO(LOGGER, "Named target available: %s", name.c_str());
    }

    // Try to move the robot arm to the predefined "Gestures" pose. If the motion fails, print an error and stop execution
    if (!move_to_named_target("Gestures")) {
      RCLCPP_ERROR(LOGGER, "Failed to move arm to Gestures pose");
      return;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Creates a dynamic array of strings for hand gestures.
    std::vector<std::string> gesture_names = {
      "fully_open",
      "fully_closed",
      "index_point",
      "middle_point",
      "victory_sign",
      "fully_open"
    };

    // Loop through all gestures
    for (const auto &gesture : gesture_names) {
      RCLCPP_INFO(LOGGER, "Calling hand gesture: %s", gesture.c_str());

      // calling ros2 service
      if (!call_hand_service("gesture", gesture)) {
        RCLCPP_WARN(LOGGER, "Gesture failed: %s", gesture.c_str());
      }

      rclcpp::sleep_for(std::chrono::seconds(2));
    }

    RCLCPP_INFO(LOGGER, "Gesture sequence completed... Moving arm back to home pose");

    if (!move_to_named_target("Home")) {
      RCLCPP_ERROR(LOGGER, "Failed to move arm to home pose");
      return;
    }
  }

// Functions only accessible inside class:
private:
  // Function to move the arm to a defined pose (Check the ur5e_moveit_config/config/ur5e.srdf file)
  bool move_to_named_target(const std::string &target_name)
  {
    move_group_->setStartStateToCurrentState(); // Planner starts from CURRENT robot configuration.
    move_group_->setNamedTarget(target_name);

    MoveGroupInterface::Plan plan; // plan is a variable of type Plan, which is defined inside MoveGroupInterface
    bool success = static_cast<bool>(move_group_->plan(plan)); // plan motion (static_cast<bool> converts a value into true/false)

    // If planning failed - print Error
    if (!success) {
      RCLCPP_ERROR(LOGGER, "Planning failed named target: %s", target_name.c_str());
      return false;
    }

    // If planning is successful, execute the motion
    RCLCPP_INFO(LOGGER, "Plan successful for %s, executing...", target_name.c_str());
    auto result = move_group_->execute(plan);

    move_group_->clearPoseTargets(); // Clears old goals.

    // Check if MoveIt operation failed (returned anything other than SUCCESS)
    if (result != MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "Execution failed for %s joint pose", target_name.c_str());
      return false;
    }

    RCLCPP_INFO(LOGGER, "Arm reached %s joint pose successfully.", target_name.c_str());
    return true;
  }

  // This function calls the RH8D hand controller:
  bool call_hand_service(const std::string &mode, const std::string &gesture_name)
  {
    if (!hand_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(LOGGER, "Hand service /rh8d/hand_command not available");
      return false;
    }

    // Check the rh8d_mujoco_interfaces/srv/HandCommand.srv file
    auto request = std::make_shared<HandCommand::Request>(); // Creates a shared pointer to HandCommand::Request object
    request->mode = mode;
    request->gesture_name = gesture_name;

    // Send the hand command request asynchronously and get a future object
    auto future = hand_client_->async_send_request(request);

    // If the response is not ready within 20 seconds, log an error and return false
    auto status = future.wait_for(std::chrono::seconds(20));
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(LOGGER, "Timed out waiting for hand service response");
      return false;
    }

    // Retrieve the actual response from the future once it is ready
    auto response = future.get();
    RCLCPP_INFO(LOGGER, "Hand response: success=%s, message=%s",
                response->success ? "true" : "false",
                response->message.c_str());
    
    // Return true if the hand executed successfully, false otherwise
    return response->success;
  }

  // This function creates obstacles:
  void add_table_collision()
  {
    // Create a dynamic list (resizeable) called collision_objects that can store multiple CollisionObject messages.
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    const std::string frame_id = move_group_->getPlanningFrame();

    // Lambda function is an anonymous function which is defined inline. General: [ capture list ] ( parameters ) { body }
    // [&] → capture everything by reference from the surrounding scope
    auto make_box = [&](const std::string& id,
                        double x, double y, double z,
                        double px, double py, double pz)
    {
      // Create a CollisionObject
      moveit_msgs::msg::CollisionObject obj;
      obj.id = id;
      obj.header.frame_id = frame_id;

      // Define the shape (Box)
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {x, y, z};

      // Define the position and orientation
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = px;
      pose.position.y = py;
      pose.position.z = pz;

      // Add shape and pose to object
      obj.primitives.push_back(primitive);
      obj.primitive_poses.push_back(pose);
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;

      return obj;
    };

    // MoveIt uses full box dimensions, MuJoCo uses half-size
    collision_objects.push_back(
      make_box("stand_base_col",
               2.0, 0.72, 0.40,
               0.1, -0.137, 0.5301));

    collision_objects.push_back(
      make_box("stand_column_col",
               0.10, 0.18, 0.80,
               0.0, 0.135, 1.045));

    collision_objects.push_back(
      make_box("stand_top_col",
               0.48, 0.182, 0.052,
               0.0, 0.135, 1.42));

    collision_objects.push_back(
      make_box("stand_back_col",
               2.0, 0.005, 0.60,
               0.1, 0.227, 1.03));

    collision_objects.push_back(
      make_box("stand_right_side_col",
               0.005, 0.725, 0.60,
               1.1, -0.135, 1.03));

    planning_scene_interface_.applyCollisionObjects(collision_objects); // Apply all collision objects in the vector to MoveIt planning scene
    RCLCPP_INFO(LOGGER, "Added collision_objectstable collision objects to MoveIt");
  }

  std::shared_ptr<MoveGroupInterface> move_group_; // Smart pointer to control the robot arm (MoveGroupInterface)
  PlanningSceneInterface planning_scene_interface_; // a variable of type 'PlanningSceneInterface' to manage the planning scene (obstacles)
  
  // hand_client_ is a smart pointer to a ROS2 service client for the HandCommand service
  rclcpp::Client<HandCommand>::SharedPtr hand_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Creates an instance of the class 'UR5eGestureDemo' as a smart pointer
  auto node = std::make_shared<UR5eGestureDemo>();

  // Creates a ROS2 executor that will manage callbacks (subscriptions, timers, service responses) and everything runs in one thread, in order.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Run executor in a separate thread to handle ROS2 callbacks continuously
  // [&executor] = capture the existing executor by reference so the lambda uses the real executor object
  // spinner is constructed directly with the lambda as the argument --- General form: Type variable(constructor_args); 
  std::thread spinner([&executor]() { executor.spin(); });

  rclcpp::sleep_for(std::chrono::seconds(2));
  node->run(); // run the main function of the class

  rclcpp::shutdown();
  spinner.join();
  return 0;
}