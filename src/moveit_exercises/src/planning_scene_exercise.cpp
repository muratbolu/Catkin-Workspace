
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <eigen_conversions/eigen_msg.h>

bool stateFeasibilityTestExample(const moveit::core::RobotState& kinematic_state, bool /*verbose*/) {
  const double* joint_values = kinematic_state.getJointPositions("panda_joint1");
  return (joint_values[0] > 0.0);
}

std::string printer(const std::vector<std::string>  &vec) {

    std::stringstream output;
    size_t size = vec.size();
    output << "Variable Names: vector = [";
    for (int i = 0; i < size; i++) {
        output << vec[i];
        if (i < (size - 1)) output << ", ";
    }
    output << "]";
    return output.str();

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "panda_arm_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.loadRobotStatePub("my_robot_state", true);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // Collision checking
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    collision_request.group_name = "hand";
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    std::vector<double> joint_values = {0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0};
    const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO_STREAM("Test 4: Current state is " << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
    
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it) {
        ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }

    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    moveit::core::RobotState copied_state = planning_scene.getCurrentState();
    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); it2++) {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Kinematic Constraints
    std::string end_effector_name = joint_model_group->getLinkModelNames().back();
    geometry_msgs::PoseStamped desired_pose;
    desired_pose.pose.orientation.w = 1.0;
    desired_pose.pose.position.x = 0.3;
    desired_pose.pose.position.y = -0.185;
    desired_pose.pose.position.z = 0.5;
    desired_pose.header.frame_id = "panda_link0";
    moveit_msgs::Constraints goal_constraint =
        kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);
    copied_state.setToRandomPositions();
    copied_state.update();
    bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
    ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));
    kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
    kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
    bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
    ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not constrained"));
    kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
        kinematic_constraint_set.decide(copied_state);
    ROS_INFO_STREAM("Test 10: Random state is " << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));
    
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    current_state.update();
    Eigen::Isometry3d transform = current_state.getFrameTransform(end_effector_name);
    tf::poseEigenToMsg(transform, desired_pose.pose);
    desired_pose.header.frame_id = kinematic_model->getRootLink()->getName();
    moveit_msgs::Constraints goal_constraint_2 =
        kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);
    bool constrained_3 = planning_scene.isStateConstrained(current_state, goal_constraint_2, true);
    ROS_INFO_STREAM("Test 10.5: Current state is " << (constrained_3 ? "constrained" : "not constrained"));

    // User-defined constraints
    planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
    bool state_feasible = planning_scene.isStateFeasible(copied_state);
    ROS_INFO_STREAM("Test 11: Random state is " << (state_feasible ? "feasible" : "not feasible"));
    bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "panda_arm");
    ROS_INFO_STREAM("Test 12: Random state is " << (state_valid ? "valid" : "not valid"));

    ros::shutdown();
    return 0;

}