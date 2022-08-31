
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

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();
    moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
        planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
    const moveit::core::JointModelGroup* joint_model_group =
        robot_state->getJointModelGroup("panda_arm");
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(
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
    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints(
            "panda_link8", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);
    /*{
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    planning_pipeline->generatePlan(lscene, req, res);
    }*/
    if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }
    
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to finish the demo");

    ROS_INFO("Done");
    return 0;

}
