//-------------------------------------------------------------------------------------------
/*! \file    state_validity_checker_node.cpp
    \brief   State validity checker node.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.27, 2015

    State validity checker node that has an interface:
      To add objects to the scene.
      To attach objects to the robot.
      To check the state validity that considers collision, constraints, and feasibility.
      To check the joint trajectory validity.

    References:
      http://moveit.ros.org/documentation/tutorials/
      http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_tutorial.html
      https://github.com/ros-planning/moveit_pr2/tree/hydro-devel/pr2_moveit_tutorials/planning
      http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html
      https://github.com/ros-planning/moveit_pr2/blob/groovy-devel/pr2_moveit_tutorials/planning/src/planning_scene_ros_api.cpp
      http://docs.ros.org/groovy/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html
      http://docs.ros.org/hydro/api/moveit_core/html/collision__tools_8h.html
      /opt/ros/groovy/share/moveit_msgs/msg/PlanningScene.msg
*/
//-------------------------------------------------------------------------------------------
#include "lfd_trick/SetPlanningSceneDiff.h"
#include "lfd_trick/GetTrajectoryValidity.h"
#include "lfd_trick/GetAllowedCollisionMatrix.h"
#include "lfd_trick/GetStateValidity.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
// For updating transforms
// #include <tf/transform_listener.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit_msgs/PlanningScene.h>
#include <eigen_conversions/eigen_msg.h>
//-------------------------------------------------------------------------------------------
namespace trick
{

// Base planning scene:
planning_scene::PlanningScenePtr PlanningSceneParent;
// Base planning scene + additional objects actually used to check the validity:
planning_scene::PlanningScenePtr PlanningScene;

// For updating transforms:
// boost::shared_ptr<tf::TransformListener> TFListener;


void CallbackJointStates(const sensor_msgs::JointState &msg)
{
  // Update current state:
  robot_state::RobotState& current_state= PlanningSceneParent->getCurrentStateNonConst();
  robot_state::jointStateToRobotState(msg, current_state);

  // Update transforms from tf
  #if 0
  robot_state::Transforms &transforms(PlanningSceneParent->getTransformsNonConst());
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped transform_msg;
  const std::string &plan_frame(PlanningSceneParent->getPlanningFrame());
  try
  {
    TFListener->lookupTransform("/base_footprint", plan_frame, ros::Time(0), transform);
    tf::transformStampedTFToMsg(transform, transform_msg);
    // ROS_INFO_STREAM("transform_msg of '/base_footprint':\n"<<transform_msg);
    transforms.setTransform(transform_msg);

    TFListener->lookupTransform("/base_link", plan_frame, ros::Time(0), transform);
    tf::transformStampedTFToMsg(transform, transform_msg);
    // ROS_INFO_STREAM("transform_msg of '/base_link':\n"<<transform_msg);
    transforms.setTransform(transform_msg);

    TFListener->lookupTransform("/torso_lift_link", plan_frame, ros::Time(0), transform);
    tf::transformStampedTFToMsg(transform, transform_msg);
    // ROS_INFO_STREAM("transform_msg of '/torso_lift_link':\n"<<transform_msg);
    transforms.setTransform(transform_msg);
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  #endif
}
//-------------------------------------------------------------------------------------------

bool SetPlanningSceneDiff(lfd_trick::SetPlanningSceneDiff::Request &req, lfd_trick::SetPlanningSceneDiff::Response &res)
{
  PlanningScene= PlanningSceneParent->diff(req.planning_scene_diff);
  ROS_INFO_STREAM("PlanningFrame: "<<PlanningScene->getPlanningFrame());
  ROS_INFO_STREAM("Known Objects:");
  PlanningScene->printKnownObjects(std::cout);
  // ROS_INFO_STREAM("getTransform('/odom_combined'):\n"<<PlanningScene->getTransforms().getTransform("/odom_combined").matrix());
  // ROS_INFO_STREAM("getTransform('/base_footprint'):\n"<<PlanningScene->getTransforms().getTransform("/base_footprint").matrix());
  // ROS_INFO_STREAM("getTransform('/base_link'):\n"<<PlanningScene->getTransforms().getTransform("/base_link").matrix());
  // ROS_INFO_STREAM("getTransform('/torso_lift_link'):\n"<<PlanningScene->getTransforms().getTransform("/torso_lift_link").matrix());
  // PlanningScene->getTransformsNonConst()->setTransform()

  return true;
}
//-------------------------------------------------------------------------------------------

bool GetStateValidity(lfd_trick::GetStateValidity::Request &req, lfd_trick::GetStateValidity::Response &res)
{
  /*
  moveit_msgs::RobotState robot_state
  string group_name
  moveit_msgs::Constraints constraints
  string contacts_frame_id
  ---
  bool valid
  moveit_msgs::ContactInformation[] contacts
  moveit_msgs::CostSource[] cost_sources
  moveit_msgs::ConstraintEvalResult[] constraint_result
  */
  bool verbose(false);
  // res.valid= PlanningScene->isStateValid(req.robot_state, req.constraints, req.group_name, verbose);
  robot_state::RobotState state(PlanningScene->getCurrentState());
  //*dbg*/ROS_INFO_STREAM("DEBUG-1");
  //*dbg*/ROS_INFO_STREAM(req.robot_state);
  robot_state::robotStateMsgToRobotState(PlanningScene->getTransforms(), req.robot_state, state);
  //*dbg*/ROS_INFO_STREAM("getTransform("<<req.contacts_frame_id<<"):\n"<<PlanningScene->getTransforms().getTransform(req.contacts_frame_id).matrix());
  //*dbg*/ROS_INFO_STREAM("Is dirty?"<<(state.dirtyLinkTransforms()?"true":"false"));
  //*dbg*/state.printDirtyInfo();
  // return isStateValid(state, constr, group, verbose);

  res.valid= true;

  // Collision check:
  // PlanningScene->isStateColliding(state, req.group_name, verbose)
  collision_detection::CollisionRequest col_req;
  col_req.verbose= verbose;
  col_req.group_name= req.group_name;
  col_req.contacts= true;  // WARNING: may be heavy
  col_req.max_contacts= 1000;  // WARNING: may be heavy
  collision_detection::CollisionResult  col_res;
  PlanningScene->checkCollision(col_req, col_res, state);
  if(col_res.collision)
  {
    res.valid= false;
    moveit_msgs::ContactInformation contact_info;
    // contact_info.header.frame_id= PlanningScene->getPlanningFrame();
    contact_info.header.frame_id= req.contacts_frame_id;  // "/torso_lift_link";
    for(collision_detection::CollisionResult::ContactMap::const_iterator itr(col_res.contacts.begin()),itr_end(col_res.contacts.end());
        itr!=itr_end; ++itr)
    {
      const std::vector<collision_detection::Contact> &contacts(itr->second);
      for(std::vector<collision_detection::Contact>::const_iterator citr(contacts.begin()),citr_end(contacts.end());
          citr!=citr_end; ++citr)
      {
        collision_detection::contactToMsg(*citr, contact_info);
        // Transform vectors:
        //*dbg*/ROS_INFO_STREAM("DEBUG-2");
        Eigen::Affine3d transform(PlanningScene->getTransforms().getTransform(contact_info.header.frame_id).inverse());
        //*dbg*/ROS_INFO_STREAM("getTransform("<<contact_info.header.frame_id<<").inverse():\n"<<PlanningScene->getTransforms().getTransform(contact_info.header.frame_id).inverse().matrix());
        //*dbg*/ROS_INFO_STREAM("Is dirty?"<<(state.dirtyLinkTransforms()?"true":"false"));
        //*dbg*/state.printDirtyInfo();
        tf::pointEigenToMsg((transform * Eigen::Translation3d(citr->pos)).translation(), contact_info.position);
        tf::vectorEigenToMsg(transform.rotation() * citr->normal, contact_info.normal);
        res.contacts.push_back(contact_info);
      }
    }
    // FIXME:it's better to omit copying cost source:
    moveit_msgs::CostSource cost_src;
    for(std::set<collision_detection::CostSource>::const_iterator itr(col_res.cost_sources.begin()),itr_end(col_res.cost_sources.end());
        itr!=itr_end; ++itr)
    {
      collision_detection::costSourceToMsg(*itr, cost_src);
      res.cost_sources.push_back(cost_src);
    }
    return true;
  }

  // Feasibility check:
  if(!PlanningScene->isStateFeasible(state, verbose))
  {
    res.valid= false;
    return true;
  }

  // Constraint check:
  // PlanningScene->isStateConstrained(state, req.constraints, verbose)
  kinematic_constraints::KinematicConstraintSetPtr ks(new kinematic_constraints::KinematicConstraintSet(PlanningScene->getRobotModel()));
  ks->add(req.constraints, PlanningScene->getTransforms());
  if(!ks->empty())
  {
    // Light weight:
    // kinematic_constraints::ConstraintEvaluationResult constr_result= ks->decide(state, verbose);
    // if(!constr_result.satisfied)
    // {
      // res.valid= false;
      // moveit_msgs::ConstraintEvalResult  c_info;
      // c_info.result= constr_result.satisfied;
      // c_info.distance= constr_result.distance;
      // res.constraint_result.push_back(c_info)
      // return true;
    // }
    // Heavier:
    std::vector<kinematic_constraints::ConstraintEvaluationResult>  constr_results;
    kinematic_constraints::ConstraintEvaluationResult constr_result= ks->decide(state, constr_results, verbose);
    if(!constr_result.satisfied)
    {
      res.valid= false;
      moveit_msgs::ConstraintEvalResult  c_info;
      for(std::vector<kinematic_constraints::ConstraintEvaluationResult>::const_iterator
          itr(constr_results.begin()), itr_end(constr_results.end()); itr!=itr_end; ++itr)
      {
        c_info.result= itr->satisfied;
        c_info.distance= itr->distance;
        res.constraint_result.push_back(c_info);
      }
      return true;
    }
  }

  return true;
}
//-------------------------------------------------------------------------------------------

bool GetTrajectoryValidity(lfd_trick::GetTrajectoryValidity::Request &req, lfd_trick::GetTrajectoryValidity::Response &res)
{
  /*
  moveit_msgs/RobotState  start_state
  moveit_msgs/RobotTrajectory  trajectory
  moveit_msgs/Constraints  path_constraints
  #string group
  ---
  bool valid
  uint32[] invalid_index
  */
  bool verbose(false);
  std::vector<std::size_t> invalid_index;
  res.valid= PlanningScene->isPathValid(
                  req.start_state,
                  req.trajectory,
                  req.path_constraints,
                  /*group=*/"", verbose, &invalid_index);
  res.invalid_index.resize(invalid_index.size());
  std::copy(invalid_index.begin(),invalid_index.end(), res.invalid_index.begin());
  return true;
}
//-------------------------------------------------------------------------------------------

bool GetAllowedCollisionMatrix(lfd_trick::GetAllowedCollisionMatrix::Request &req, lfd_trick::GetAllowedCollisionMatrix::Response &res)
{
  PlanningSceneParent->getAllowedCollisionMatrix().getMessage(res.allowed_collision_matrix);
  return true;
}
//-------------------------------------------------------------------------------------------

}  // end of trick
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  using namespace trick;

  ros::init(argc, argv, "state_validity_checker");
  ros::NodeHandle node("~");
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // TF listener:
  // TFListener= boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());

  // Setup the planning scene from robot model:
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model= robot_model_loader.getModel();
  PlanningSceneParent= planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model));
  PlanningScene= PlanningSceneParent->diff();

  // Start providing services:
  ros::ServiceServer srv_set_planning_scene_diff= node.advertiseService("set_planning_scene_diff", &SetPlanningSceneDiff);
  ros::ServiceServer srv_get_state_validity= node.advertiseService("get_state_validity", &GetStateValidity);
  ros::ServiceServer srv_get_trajectory_validity= node.advertiseService("get_trajectory_validity", &GetTrajectoryValidity);
  ros::ServiceServer srv_get_allowed_collision_matrix= node.advertiseService("get_allowed_collision_matrix", &GetAllowedCollisionMatrix);

  // Joint state updater:
  ros::Subscriber sub_joint_states= node.subscribe("/joint_states", 1, &CallbackJointStates);

  ros::spin();
  return 0;
}
//-------------------------------------------------------------------------------------------
