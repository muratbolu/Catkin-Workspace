
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

const double tau = 2 * M_PI;

int main(int argc, char** argv) {

    // Initialization
    ros::init(argc, argv, "move_group_interface_exercise");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Setup
    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Print basic information
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // Start demo, pose goal
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.5;
    target_pose1.position.y = 0.5;
    target_pose1.position.z = 0.5;
    move_group_interface.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Joint goal
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = -tau / 4;
    move_group_interface.setJointValueTarget(joint_group_positions);
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Joint constraint
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "panda_link7";
    ocm.header.frame_id = "panda_link0";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_interface.setPathConstraints(test_constraints);
    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group_interface.setStartState(start_state);
    move_group_interface.setPoseTarget(target_pose1);
    move_group_interface.setPlanningTime(10.0);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start");
    visual_tools.publishAxisLabeled(target_pose1, "goal");
    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");
    move_group_interface.clearPathConstraints();

    // Cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);
    geometry_msgs::Pose target_pose3 = start_pose2;
    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3);
    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3);
    target_pose3.position.z += 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    double fraction = move_group_interface.computeCartesianPath(waypoints,
                                                                eef_step,
                                                                jump_threshold,
                                                                trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    // Seems to give an error:
    move_group_interface.execute(trajectory);

    // Obstacle
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    geometry_msgs::Pose another_pose;
    another_pose.orientation.x = 1.0;
    another_pose.position.x = 0.7;
    another_pose.position.y = 0.0;
    another_pose.position.z = 0.59;
    move_group_interface.setPoseTarget(another_pose);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("next step");
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    // Attach object
    moveit_msgs::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";
    shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;
    object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
    geometry_msgs::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.2;
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    move_group_interface.attachObject(object_to_attach.id, "panda_hand");
    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");
    move_group_interface.setStartStateToCurrentState();
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    // Detach and remove objects
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    move_group_interface.detachObject(object_to_attach.id);
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");
    ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    object_ids.push_back(object_to_attach.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

    ros::shutdown();
    return 0;

}
