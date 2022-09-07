
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "move_group_exercise");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(
        "robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(
        robot_model_loader));
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();
    moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
        planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("panda_arm");
    planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(
        robot_model, node_handle, "planning_plugin", "request_adapters"));

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(
        text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Pose goal
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    req.group_name = "panda_arm";
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
        "panda_link8", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }
    if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
        "/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Joint goal
    robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(
        response.trajectory_start);
    robot_state->setJointGroupPositions(joint_model_group,
        response.trajectory.joint_trajectory.points.back().positions);
    moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
    moveit::core::RobotState goal_state(*robot_state);
    std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(
        goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }
    if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(
        response.trajectory_start);
    robot_state->setJointGroupPositions(joint_model_group,
        response.trajectory.joint_trajectory.points.back().positions);
    moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
    const moveit::core::JointModel* joint_model = joint_model_group->getJointModel("panda_joint3");
    const moveit::core::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
    std::vector<double> tmp_values(1, 0.0);
    tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
    robot_state->setJointPositions(joint_model, tmp_values);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }
    if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");

    ROS_INFO("Done");
    return 0;

}
