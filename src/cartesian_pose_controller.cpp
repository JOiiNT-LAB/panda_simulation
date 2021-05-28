#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <cstring>
#include <iostream>
using namespace std;
#include <stdio.h>
using namespace KDL;

namespace panda_simulation {

class CartesianPoseController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
  bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n) {

    printf("Starting control node \n ");
    std::vector<std::string> joint_names;
    if (!n.getParam("joint_names", joint_names)) {
      ROS_ERROR("Could not read joint names from param server");
      return false;
    }

    // retrieve gains
    if (!n.getParam("gains", gains_vec_)) {
      ROS_ERROR("Could not read joint gains from param server");
      return false;
    }

    for (auto &joint_name : joint_names) {
      joint_handles_.push_back(hw->getHandle(joint_name));
    }

    for (auto &joint_handle : joint_handles_) {
      command_.push_back(joint_handle.getPosition());
    }

    ////////////////////////////////////////////////////////////////////////////
    /////////////////     DEFINE ROBOT CHAIN          /////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string robot_description, root_name, tip_name;
    KDL::Tree kdl_tree_;
    unsigned int num_joints;
    if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
            {
                ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
                return false;
            }
   std::string xml_string;
   if (n.hasParam(robot_description))
       n.getParam(robot_description.c_str(), xml_string);
   else
   {
       ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
       n.shutdown();
       return false;
   }
   if (!n.getParam("root_name", root_name))
           {
               ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
               return false;
           }
  
   if (!n.getParam("tip_name", tip_name))
           {
               ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
               return false;
           }

   if (xml_string.size() == 0)
           {
               ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
               n.shutdown();
               return false;
           }
    ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());

    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        n.shutdown();
        return false;
    }
    ROS_INFO("Successfully parsed urdf file");

    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        n.shutdown();
        return false;
    }
    printf("root name %s \n",root_name.c_str());
    printf("tip name %s \n",tip_name.c_str());
    printf("robot name %s \n",robot_description.c_str());
    printf("tree size %i \n",kdl_tree_.getNrOfJoints());
    printf("tree n segments %i \n",kdl_tree_.getNrOfSegments());

    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
          ROS_ERROR_STREAM( "    "<<(*it).first);
        printf("get chain failed");
        return false;
    }

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>(std::string("command"), 1,
                                                            &CartesianPoseController::setCommandCallback, this);//\brief robot_description
    
 
    return true;
  }
  
  

  void update(const ros::Time &time, const ros::Duration &period) {
    
    geometry_msgs::PoseStamped pose_msg_in;
    pose_msg_in.header.frame_id="/ee";
    pose_msg_in.header.stamp = ros::Time::now();
    pose_msg_in.pose.position.x = 0.0;
    pose_msg_in.pose.position.y = 0.0;
    pose_msg_in.pose.position.z = 0.3;
    pose_msg_in.pose.orientation.x = 0.0;
    pose_msg_in.pose.orientation.y = 0.0;
    pose_msg_in.pose.orientation.z = 0.0;
    pose_msg_in.pose.orientation.w = 1.0;

    for (size_t i = 0; i < joint_handles_.size(); i++) {
    
    getinversekinematic(pose_msg_in);
      
      command_.at(i) = jnt_des_ik(i);
      double error = command_.at(i) - joint_handles_.at(i).getPosition();
      //std::cout<<jnt_des_ik(i)<<std::endl;
      double commanded_effort = error * gains_vec_.at(i);

      joint_handles_.at(i).setCommand(commanded_effort);
      
    }
  }

  void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg) {

    

    //getinversekinematic(pose_msg_in); 

    //for (size_t i = 0; i < joint_handles_.size(); i++) {
    //  command_.at(i) = jnt_des_ik(i);
    //}

    }

  void starting(const ros::Time &time) {}

  void stopping(const ros::Time &time) {}
  
  void getinversekinematic(const geometry_msgs::PoseStamped &pose_msg_in){

    tf::Stamped<tf::Pose> transform;
    tf::Stamped<tf::Pose> transform_root;
    tf::poseStampedMsgToTF( pose_msg_in, transform );
    //std::cout<<pose_msg_in<<std::endl;
    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fk_solver_ = ChainFkSolverPos_recursive(kdl_chain_);
    ChainIkSolverVel_pinv ik_solver_vel_ = ChainIkSolverVel_pinv(kdl_chain_);
    ChainIkSolverPos_NR ik_solver_(kdl_chain_, fk_solver_, ik_solver_vel_, 1000, 100);

    tf::TransformListener tf_listener;
    KDL::JntArray jnt_pos_in((kdl_chain_.getNrOfJoints()));
    jnt_des_ik.resize(kdl_chain_.getNrOfJoints());

    //Convert F to our root_frame
    //tf_listener.transformPose(root_name, transform, transform_root);
       
    KDL::Frame F_dest;
    //tf::TransformTFToKDL(transform_root, F_dest); // origin,end_effector_pose,result
    tf::TransformTFToKDL(transform, F_dest); // origin,end_effector_pose,result

    //printf("chain size %i \n",kdl_chain_.getNrOfJoints());

    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
            jnt_pos_in(i) = joint_handles_[i].getPosition();
        }

    //printf("init joints: \n");
    //for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
    //        printf("%f \n",jnt_pos_in(i));
    //    }

    //printf("Starting direct kinematic \n");
    //bool kinematics_status1;
    //kinematics_status1 = fk_solver_.JntToCart(jnt_pos_in,F_dest);
    //if(kinematics_status1>=0){
    //    printf("%s \n","Succes fk, thanks KDL!");
    //}else{
    //    printf("%s \n","Error: could not calculate forward kinematics :(");
    //}


    //printf("Starting inverse kinematic \n");
    bool kinematics_status;
    kinematics_status = ik_solver_.CartToJnt(jnt_pos_in,F_dest,jnt_des_ik);
   // if (kinematics_status>=0){
       //printf("Succesfull IK \n");
   //    }
    //printf("Inverse kinematic completed \n");

    //printf("desired joints: \n");
    //for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++){
    //        printf("%f \n",jnt_des_ik(i));
    //    }
  }

private:
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<double> gains_vec_;
  std::vector<double> command_;
  ros::Subscriber sub_command_;
  KDL::Chain kdl_chain_;
  KDL::JntArray jnt_des_ik;
};

PLUGINLIB_EXPORT_CLASS(panda_simulation::CartesianPoseController, controller_interface::ControllerBase);

} // namespace panda_simulation
