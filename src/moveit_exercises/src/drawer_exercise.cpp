
#include <ros/ros.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "drawer_exercise");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node;

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    static const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    moveit_msgs::PlanningScene planning_scene;

    ros::Publisher publisher = node.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    ros::WallDuration sleep_t(0.5);
    while (publisher.getNumSubscribers() < 1) sleep_t.sleep();

    // Create box object
    moveit_msgs::CollisionObject box;
    box.header.frame_id = robot_model->getModelFrame();
    box.id = "box";

    shape_msgs::SolidPrimitive box_primitive;
    box_primitive.type = box_primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = 0.075;
    box_primitive.dimensions[1] = 0.075;
    box_primitive.dimensions[2] = 0.075;

    geometry_msgs::Pose box_pose;
    box_pose.position.y = 0.5;

    box.primitives.push_back(box_primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;

    planning_scene.world.collision_objects.push_back(box);
    planning_scene.is_diff = true;

    // Create drawer object
    moveit_msgs::CollisionObject drawer;
    drawer.header.frame_id = robot_model->getModelFrame();
    drawer.id = "drawer";

    geometry_msgs::Pose drawer_pose;
    drawer_pose.position.x = 1;
    drawer_pose.position.z = 0.1;

    shape_msgs::SolidPrimitive left_side_primitive;
    left_side_primitive.type = left_side_primitive.BOX;
    left_side_primitive.dimensions.resize(3);
    left_side_primitive.dimensions[0] = 0.2;
    left_side_primitive.dimensions[1] = 0.01;
    left_side_primitive.dimensions[2] = 0.2;

    geometry_msgs::Pose left_side_pose;
    left_side_pose.position.x = 0.6;
    left_side_pose.position.y = 0.1;
    left_side_pose.position.z = 0.5;

    shape_msgs::SolidPrimitive right_side_primitive;
    right_side_primitive.type = right_side_primitive.BOX;
    right_side_primitive.dimensions.resize(3);
    right_side_primitive.dimensions[0] = 0.2;
    right_side_primitive.dimensions[1] = 0.01;
    right_side_primitive.dimensions[2] = 0.2;

    geometry_msgs::Pose right_side_pose;
    right_side_pose.position.x = 0.6;
    right_side_pose.position.y = -0.1;
    right_side_pose.position.z = 0.5;

    shape_msgs::SolidPrimitive down_side_primitive;
    down_side_primitive.type = down_side_primitive.BOX;
    down_side_primitive.dimensions.resize(3);
    down_side_primitive.dimensions[0] = 0.2;
    down_side_primitive.dimensions[1] = 0.2;
    down_side_primitive.dimensions[2] = 0.01;

    geometry_msgs::Pose down_side_pose;
    down_side_pose.position.x = 0.6;
    down_side_pose.position.y = 0;
    down_side_pose.position.z = 0.4;

    shape_msgs::SolidPrimitive up_side_primitive;
    up_side_primitive.type = up_side_primitive.BOX;
    up_side_primitive.dimensions.resize(3);
    up_side_primitive.dimensions[0] = 0.2;
    up_side_primitive.dimensions[1] = 0.2;
    up_side_primitive.dimensions[2] = 0.01;

    geometry_msgs::Pose up_side_pose;
    up_side_pose.position.x = 0.6;
    up_side_pose.position.y = 0;
    up_side_pose.position.z = 0.6;

    shape_msgs::SolidPrimitive back_side_primitive;
    back_side_primitive.type = back_side_primitive.BOX;
    back_side_primitive.dimensions.resize(3);
    back_side_primitive.dimensions[0] = 0.01;
    back_side_primitive.dimensions[1] = 0.2;
    back_side_primitive.dimensions[2] = 0.2;

    geometry_msgs::Pose back_side_pose;
    back_side_pose.position.x = 0.7;
    back_side_pose.position.y = 0;
    back_side_pose.position.z = 0.5;

    drawer.primitives.push_back(left_side_primitive);
    drawer.primitives.push_back(right_side_primitive);
    drawer.primitives.push_back(down_side_primitive);
    drawer.primitives.push_back(up_side_primitive);
    drawer.primitives.push_back(back_side_primitive);
    drawer.primitive_poses.push_back(left_side_pose);
    drawer.primitive_poses.push_back(right_side_pose);
    drawer.primitive_poses.push_back(down_side_pose);
    drawer.primitive_poses.push_back(up_side_pose);
    drawer.primitive_poses.push_back(back_side_pose);
    drawer.operation = drawer.ADD;

    planning_scene.world.collision_objects.push_back(drawer);
    planning_scene.is_diff = true;
    publisher.publish(planning_scene);

    ros::shutdown();
    return 0;

}