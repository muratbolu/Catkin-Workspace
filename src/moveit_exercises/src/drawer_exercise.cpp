
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

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
        box_pose.position.z = position[2];
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
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.loadRemoteControl();

    const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    const std::string inertial_frame = robot_model->getModelFrame();

    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    // Create box object and add it to the planning scene
    moveit_msgs::CollisionObject box = CreateBox(inertial_frame);
    planning_scene->processCollisionObjectMsg(box);

    // Create drawer object and add it to the planning scene
    moveit_msgs::CollisionObject drawer = CreateBox(inertial_frame);
    planning_scene->processCollisionObjectMsg(drawer);

    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan and execute the trajectory");
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    if (!node.getParam("planning_plugin", planner_plugin_name)) ROS_FATAL_STREAM("Could not find planner plugin name");
    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, node.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes) ss << cls << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': "
                                                             << ex.what() << std::endl
                                                             << "Available plugins: " << ss.str());
    }

    ros::shutdown();
    return 0;

}