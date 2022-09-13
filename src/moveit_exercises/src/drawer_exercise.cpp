
#include <ros/ros.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

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
    ros::NodeHandle node;

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    static const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    moveit_msgs::PlanningScene planning_scene;
    static const std::string inertial_frame = robot_model->getModelFrame();

    ros::Publisher publisher = node.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    ros::WallDuration sleep_t(0.5);
    while (publisher.getNumSubscribers() < 1) sleep_t.sleep();

    // Create box object and add it to the planning scene
    planning_scene.world.collision_objects.push_back(moveit_msgs::CollisionObject(CreateBox(inertial_frame)));
    planning_scene.is_diff = true;

    // Create drawer object and add it to the planning scene
    planning_scene.world.collision_objects.push_back(moveit_msgs::CollisionObject(CreateDrawer(inertial_frame)));
    planning_scene.is_diff = true;

    publisher.publish(planning_scene);

    ros::shutdown();
    return 0;

}