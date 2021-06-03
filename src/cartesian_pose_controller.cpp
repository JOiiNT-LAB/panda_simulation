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
#include <eigen3/Eigen/LU>
#include <cstring>
#include <iostream>
using namespace std;
#include <stdio.h>
using namespace KDL;
#include <panda_simulation/PoseRPY.h>

#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

namespace panda_simulation {

class CartesianPoseController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
  bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n) {

    std::vector<std::string> joint_names;
    if (!n.getParam("joint_names", joint_names)) {
      ROS_ERROR("Could not read joint names from param server");
      return false;
    }

     for (auto &joint_name : joint_names) {
      joint_handles_.push_back(hw->getHandle(joint_name));
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

    J_.resize(kdl_chain_.getNrOfJoints());

    joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
    joint_des_states_.resize(kdl_chain_.getNrOfJoints());
    // get joint positions
    for(int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
    }

    // computing forward kinematics
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
    
    //Desired posture is the current one
    x_des_ = x_;

    cmd_flag_ = 0;


    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
    qo_dot_.resize(kdl_chain_.getNrOfJoints());
    proj_.resize(kdl_chain_.getNrOfJoints());
    In_ = Eigen::MatrixXd::Identity(kdl_chain_.getNrOfJoints(),kdl_chain_.getNrOfJoints());
    
    for (int joint_num = 0; joint_num < kdl_chain_.getNrOfJoints(); joint_num++)
        {
            joint_limits_.max(joint_num) = joint_max_[joint_num];//joint_->limits->upper;
            joint_limits_.min(joint_num) = joint_min_[joint_num];//joint_->limits->lower;
            joint_limits_.center(joint_num) = (joint_limits_.min(joint_num) + joint_limits_.max(joint_num))/2;
            //joint_msr_states_.q(joint_num) = joint_limits_.center(joint_num);
            joint_des_states_.q(joint_num) = joint_limits_.center(joint_num);
            //std::cout << joint_limits_.center(joint_num) << " " << joint_msr_states_.q(joint_num) << "\n";
        }
    
    sub_command_ = n.subscribe("command", 1, &CartesianPoseController::setCommandCallback, this);

        return true;
  }
  
  

  void update(const ros::Time &time, const ros::Duration &period) {
    // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }
        
        if (cmd_flag_)
        {

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

            // end-effector position error
            x_err_.vel = x_des_.p - x_.p;
            // getting quaternion from rotation matrix
            x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
            x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

            skew_symmetric(quat_des_.v, skew_);

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;
                for (int k = 0; k < skew_.cols(); k++)
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
            }

            // end-effector orientation error
            x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;


            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);
            

            // computing J_pinv_
            pseudo_inverse(J_.data, J_pinv_);


            // computing Null Projection
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                proj_(i) = 0.0;
                //qo_dot_(i) = (joint_limits_.center(i)-joint_msr_states_.q(i));
                qo_dot_(i) = -pow((joint_limits_.max(i)-joint_limits_.min(i)),2)*(2*joint_msr_states_.q(i)-joint_limits_.max(i)-joint_limits_.min(i))
                            /(4*pow((joint_limits_.max(i)-joint_msr_states_.q(i)),2)*pow((joint_msr_states_.q(i)-joint_limits_.min(i)),2));
                for (int k = 0; k < J_pinv_.cols(); k++)
                {
                    proj_(i) += (In_(i,k)-J_pinv_(i,k)*J_(k,i))*qo_dot_(i)*ko_;
                }
            }
            
            // computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                {
                    joint_des_states_.qdot(i) += J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
                }
                joint_des_states_.qdot(i) += proj_(i);
            }
            
            // integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

            /*if (Equal(x_, x_des_, 0.005))
            {
                ROS_INFO("On target");
                cmd_flag_ = 0;
                printf("Position reached \n");
            }*/
        }
    
    for (int i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(joint_des_states_.q(i));
    }
  }

  void setCommandCallback(const panda_simulation::PoseRPY::ConstPtr &msg) {
    
     KDL::Frame frame_des_;

        switch(msg->id)
        {
            case 0:
            frame_des_ = KDL::Frame(
                    KDL::Rotation::RPY(msg->orientation.roll,
                                      msg->orientation.pitch,
                                      msg->orientation.yaw),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));
            break;

            case 1: // position only
            frame_des_ = KDL::Frame(
                KDL::Vector(msg->position.x,
                            msg->position.y,
                            msg->position.z));
            break;

            case 2: // orientation only
            frame_des_ = KDL::Frame(
                KDL::Rotation::RPY(msg->orientation.roll,
                                   msg->orientation.pitch,
                                   msg->orientation.yaw));
            break;

            default:
            ROS_INFO("Wrong message ID");
            return;
        }

        x_des_ = frame_des_;
        cmd_flag_ = 1;
    }

  void starting(const ros::Time &time) {
  }

  void stopping(const ros::Time &time) {}

private:
    std::vector<hardware_interface::JointHandle> joint_handles_;
    KDL::Chain kdl_chain_;
    ros::Subscriber sub_command_;

    KDL::Frame x_;		//current pose
    KDL::Frame x_des_;	//desired pose

    KDL::Twist x_err_;

    KDL::Jacobian J_;	//Jacobian

    Eigen::MatrixXd J_pinv_;
    Eigen::Matrix<double,3,3> skew_;

    struct quaternion_
    {
    KDL::Vector v;
    double a;
    } quat_curr_, quat_des_;

    KDL::Vector v_temp_;

    int cmd_flag_;

    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

    KDL::JntArrayAcc joint_msr_states_, joint_des_states_;

    KDL::JntArray qo_dot_;
    KDL::JntArray proj_;

    struct limits_
    {
        KDL::JntArray min;
        KDL::JntArray max;
        KDL::JntArray center;
    } joint_limits_;

    std::vector<float> joint_max_ = { 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973, 0.0};
    std::vector<float> joint_min_ = {-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973, 0.0};
    Eigen::MatrixXd In_;
    float ko_ = 0.1;
};

PLUGINLIB_EXPORT_CLASS(panda_simulation::CartesianPoseController, controller_interface::ControllerBase);

} // namespace panda_simulation
