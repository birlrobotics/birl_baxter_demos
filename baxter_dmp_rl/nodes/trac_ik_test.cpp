#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_conversions/kdl_msg.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndpointState.h>



std::string chain_start, chain_end, urdf_param;
double timeout = 0.005;
double eps = 1e-5;
ros::Publisher jointCommandPublisher;

// Computes IK with limits considerations. 
// Inputs: KDL::Frame        pose
//         const std::string limbName 
// Output: publishes solution.
void findIKSolution(const KDL::Frame &end_effector_pose, const std::string &limbName)
{

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, limbName + chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul;                           // lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);
  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  for (uint j=0; j<nominal.data.size(); j++) {
    nominal(j) = (ll(j)+ul(j))/2.0;
  }

  KDL::JntArray result;
  // result count
  int rc;

  rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
  ROS_INFO("Found %d solution", rc);

  if (rc > 0) {
      /* In the solution, the order of joint angles is s->e->w,
       * while baxter_core_msgs::JointCommand requires the order of e->s->w.
       * So we have to make some conversions.
      */
      std::string jointNamesArray[] = {limbName + "_s0", limbName + "_s1", limbName + "_e0", limbName + "_e1",
          limbName + "_w0", limbName + "_w1", limbName + "_w2"};
      std::vector<std::string> jointNamesVector(chain.getNrOfJoints());
      for( std::size_t j = 0; j < chain.getNrOfJoints(); ++j)
          jointNamesVector[j] = jointNamesArray[j];
      baxter_core_msgs::JointCommand jointCommand;
      jointCommand.mode = 1;
      jointCommand.names = jointNamesVector;
      jointCommand.command.resize(chain.getNrOfJoints());
      for( std::size_t j = 0; j < chain.getNrOfJoints(); ++j)
          jointCommand.command[j] = result(j);

      // The conversion has done, publishing the joint command

      jointCommandPublisher.publish(jointCommand);
  }
}

//----------------------------------------------------------------------------------------------
// Compute the Inverse Kinematics 
//----------------------------------------------------------------------------------------------
void callBack1(const geometry_msgs::PoseStamped &callBackData) {
  // ROS_INFO("Callbacking...");
  KDL::Frame end_effector_pose;
  KDL::Frame end_effector_pose_rot;
  tf::poseMsgToKDL(callBackData.pose, end_effector_pose);
  end_effector_pose_rot.p.data[0] =  end_effector_pose.p.data[0];
  end_effector_pose_rot.p.data[1] =  end_effector_pose.p.data[1];
  end_effector_pose_rot.p.data[2] =  end_effector_pose.p.data[2];
  end_effector_pose_rot.M.data[0] =  end_effector_pose.M.data[0];
  end_effector_pose_rot.M.data[1] =  end_effector_pose.M.data[1];
  end_effector_pose_rot.M.data[2] =  end_effector_pose.M.data[2];
  end_effector_pose_rot.M.data[3] =  -end_effector_pose.M.data[6];
  end_effector_pose_rot.M.data[4] =  -end_effector_pose.M.data[7];
  end_effector_pose_rot.M.data[5] =  -end_effector_pose.M.data[8];
  end_effector_pose_rot.M.data[6] =   end_effector_pose.M.data[3];
  end_effector_pose_rot.M.data[7] =   end_effector_pose.M.data[4];
  end_effector_pose_rot.M.data[8] =   end_effector_pose.M.data[5];
  // Compute the IK Solution and publish the solution from within the method
   findIKSolution(end_effector_pose_rot, "right");
}

void callBack2(const baxter_core_msgs::EndpointState &callBackData) {
  // ROS_INFO("Callbacking...");
  KDL::Frame end_effector_pose;
  tf::poseMsgToKDL(callBackData.pose, end_effector_pose);
    // Compute the IK Solution and publish the solution from within the method
  findIKSolution(end_effector_pose, "right");
  }

int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "trac_ik_test");
  ros::NodeHandle nodeHandle;

  //nodeHandle.param("chain_start", chain_start, std::string("left_arm_mount"));
  //nodeHandle.param("chain_end", chain_end, std::string("left_gripper_base"));
  //nodeHandle.param("urdf_param", urdf_param, std::string("/robot_description"));
  chain_start = "base";
  chain_end = "_gripper_base";
  urdf_param = "/robot_description";
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

//  ros::Subscriber subscriber = nodeHandle.subscribe("/aruco_tracker/pose", 1, callBack1);
  
  ros::Subscriber subscriber = nodeHandle.subscribe("/robot/limb/right/endpoint_state", 10, callBack2);
  
  jointCommandPublisher = nodeHandle.advertise<baxter_core_msgs::JointCommand>("end_effector_command_solution", 1);

  ros::spin();
  return 0;
}
