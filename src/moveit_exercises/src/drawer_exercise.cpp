
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

namespace {

    moveit_msgs::CollisionObject CreateBox(std::string inertial_frame,
                                           double edge_size = 0.075,
                                           std::vector<double> position = {0.0, 0.5, 0.0},
                                           std::vector<double> orientation = {0.0, 0.0, 0.0, 0.0}) {

        moveit_msgs::CollisionObject box;
        box.header.frame_id = inertial_frame;
        box.id = "box";

        shape_msgs::SolidPrimitive box_primitive;
        box_primitive.type = box_primitive.BOX;
        box_primitive.dimensions.resize(3);
        box_primitive.dimensions[0] = edge_size;
        box_primitive.dimensions[1] = edge_size;
        box_primitive.dimensions[2] = edge_size;

        geometry_msgs::Pose box_pose;
        box_pose.position.x = position[0];
        box_pose.position.y = position[1];
        box_pose.position.z = position[2] + edge_size/2;
        box_pose.orientation.x = orientation[0];
        box_pose.orientation.y = orientation[1];
        box_pose.orientation.z = orientation[2];
        box_pose.orientation.w = orientation[3];

        box.primitives.push_back(box_primitive);
        box.primitive_poses.push_back(box_pose);
        box.operation = box.ADD;

        return box;

    }

    moveit_msgs::CollisionObject CreateDrawer(std::string inertial_frame,
                                              double drawer_edge_size = 0.2,
                                              std::vector<double> position = {0.6, 0.0, 0.5}) {

        moveit_msgs::CollisionObject drawer;
        drawer.header.frame_id = inertial_frame;
        drawer.id = "drawer";
        drawer.operation = drawer.ADD;

        auto AddSide = [&](std::string side) {
            shape_msgs::SolidPrimitive side_primitive;
            side_primitive.type = side_primitive.BOX;
            side_primitive.dimensions.resize(3);
            side_primitive.dimensions[0] = (side == "back" ? 0.01 : drawer_edge_size);
            side_primitive.dimensions[1] = (side == "left" || side == "right" ? 0.01 : drawer_edge_size);
            side_primitive.dimensions[2] = (side ==   "up" || side ==  "down" ? 0.01 : drawer_edge_size);

            geometry_msgs::Pose side_pose;
            side_pose.position.x = position[0] + (side ==  "back" ?  drawer_edge_size/2 : 0);
            side_pose.position.y = position[1] + (side ==  "left" ?  drawer_edge_size/2 : 0)
                                               + (side == "right" ? -drawer_edge_size/2 : 0);
            side_pose.position.z = position[2] + (side ==    "up" ?  drawer_edge_size/2 : 0)
                                               + (side ==  "down" ? -drawer_edge_size/2 : 0);

            drawer.primitives.push_back(side_primitive);
            drawer.primitive_poses.push_back(side_pose);
        };

        AddSide("left");
        AddSide("right");
        AddSide("down");
        AddSide("up");
        AddSide("back");

        return drawer;

    }

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "drawer_exercise");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node("~");

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    const std::string PLANNING_GROUP = "panda_arm";
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
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(
        robot_model, node, "planning_plugin", "request_adapters"));
    const std::string inertial_frame = robot_model->getModelFrame();
    const std::string end_effector_name = joint_model_group->getLinkModelNames().back();

    // Create box and drawer objects and add it to the planning scene
    std::vector<moveit_msgs::CollisionObject> objects{CreateBox(inertial_frame), CreateDrawer(inertial_frame)};
    planning_scene_interface.applyCollisionObjects(objects);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan and execute the trajectory");

    // Pose goal
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.5;
    pose.pose.position.z = 0.15;
    Eigen::Quaterniond quat(0.0, 0.92, 0.38, 0.0);
    quat.normalize();
    tf::quaternionEigenToMsg(quat, pose.pose.orientation);
    
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    req.group_name = PLANNING_GROUP;
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
        end_effector_name, pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }
    if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    move_group_interface.execute(response.trajectory);
    move_group_interface.attachObject(objects[0].id, "panda_hand");

    // Pose goal
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.35;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.5;
    quat = Eigen::Quaterniond(-0.27, 0.65, -0.27, 0.65);
    quat.normalize();
    tf::quaternionEigenToMsg(quat, pose.pose.orientation);
    
    req.group_name = PLANNING_GROUP;
    pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name,
                                                                pose,
                                                                tolerance_pose,
                                                                tolerance_angle);
    req.goal_constraints[0] = pose_goal;
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }
    if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    res.getMessage(response);
    move_group_interface.execute(response.trajectory);
    
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the exercise");

    ros::shutdown();
    return 0;

}