
#include "ros/ros.h"
#include "ik_solver_service/SolveRollPitchIK.h"
#include "ik_solver_service/SolveClosestIK.h"
#include "ik_solver_service/SolvePreferredPitchIK.h"
#include "ik_solver_service/SolvePreferredTypeIK.h"
#include "ik_solver_service/SolveFullyConstrainedIK.h"

#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "solve_ik_client");

  ros::NodeHandle n;

  // Example for computing roll pitch IK solution
  ros::ServiceClient solve_roll_pitch_ik_client = n.serviceClient<ik_solver_service::SolveRollPitchIK>("solve_roll_pitch_ik");
  ik_solver_service::SolveRollPitchIK rp_srv;

  rp_srv.request.des_position[0] = 0.35;
  rp_srv.request.des_position[1] = 0.23;
  rp_srv.request.des_position[2] = 0.42;
  rp_srv.request.id = 1;
  rp_srv.request.roll = 0.0;
  rp_srv.request.pitch = 0.0;

  if (solve_roll_pitch_ik_client.call(rp_srv))
  {
	  printf("Solve Roll Pitch IK:\n");
	  for (int i = 0; i < 5; i++)
      {
        printf("%f\t", rp_srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", rp_srv.response.feasible,
    		  rp_srv.response.arm_to_front, rp_srv.response.arm_bended_up, rp_srv.response.gripper_downwards);
  }
  else
  {
    ROS_ERROR("Failed to call service solve_roll_pitch_ik");
    return 1;
  }

  // Example for computing closest IK solution
  ros::ServiceClient solve_closest_ik_client = n.serviceClient<ik_solver_service::SolveClosestIK>("solve_closest_ik");

  ik_solver_service::SolveClosestIK srv;

  srv.request.joint_angles[0] = 0.1; //2.9496;
  srv.request.joint_angles[1] = 0.1; //01.1345;
  srv.request.joint_angles[2] = 0.1; //-2.5482;
  srv.request.joint_angles[3] = 0.1; //1.7890;
  srv.request.joint_angles[4] = 0.1; //2.9234;

  srv.request.des_position[0] = 0.35;
  srv.request.des_position[1] = 0.0;
  srv.request.des_position[2] = 0.3;
  srv.request.des_normal[0] = 0.0;
  srv.request.des_normal[1] = 1.0;
  srv.request.des_normal[2] = 0.0;

  if (solve_closest_ik_client.call(srv))
  {
	printf("Solve Closest IK:\n");
	for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
        printf("%f\t", srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", srv.response.feasible,
             srv.response.arm_to_front, srv.response.arm_bended_up, srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_closest_ik");
    return 1;
  }

  // Example for computing preferred pitch IK solution
  ros::ServiceClient solve_preferred_pitch_ik_client = n.serviceClient<ik_solver_service::SolvePreferredPitchIK>(
      "solve_preferred_pitch_ik");

  ik_solver_service::SolvePreferredPitchIK pp_srv;

  pp_srv.request.preferred_pitch = 0.0;
  pp_srv.request.des_position[0] = 0.35;
  pp_srv.request.des_position[1] = 0.0;
  pp_srv.request.des_position[2] = 0.3;
  pp_srv.request.des_normal[0] = 0.0;
  pp_srv.request.des_normal[1] = 1.0;
  pp_srv.request.des_normal[2] = 0.0;

  if (solve_preferred_pitch_ik_client.call(pp_srv))
  {
    printf("Solve Preferred Pitch IK:\n");
	for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
        printf("%f\t", pp_srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", pp_srv.response.feasible,
             pp_srv.response.arm_to_front, pp_srv.response.arm_bended_up, pp_srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_preferred_pitch_ik");
    return 1;
  }

  // Example for computing preferred type IK solution
  ros::ServiceClient solve_preferred_type_ik_client = n.serviceClient<ik_solver_service::SolvePreferredTypeIK>(
      "solve_preferred_type_ik");

  ik_solver_service::SolvePreferredTypeIK pt_srv;

  pt_srv.request.arm_to_front = true;
  pt_srv.request.arm_bended_up = true;
  pt_srv.request.gripper_downwards = true;
  pt_srv.request.des_position[0] = 0.35;
  pt_srv.request.des_position[1] = 0.0;
  pt_srv.request.des_position[2] = 0.3;
  pt_srv.request.des_normal[0] = 0.0;
  pt_srv.request.des_normal[1] = 1.0;
  pt_srv.request.des_normal[2] = 0.0;

  if (solve_preferred_type_ik_client.call(pt_srv))
  {
	printf("Solve Preferred Type IK:\n");
	for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
        printf("%f\t", pt_srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", pt_srv.response.feasible,
             pt_srv.response.arm_to_front, pt_srv.response.arm_bended_up, pt_srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_preferred_type_ik");
    return 1;
  }

  // Example for computing fully constrained IK solution
  ros::ServiceClient solve_fully_constrained_ik_client = n.serviceClient<ik_solver_service::SolveFullyConstrainedIK>(
      "solve_fully_constrained_ik");

  ik_solver_service::SolveFullyConstrainedIK fc_srv;

  fc_srv.request.id = 1;
  fc_srv.request.pitch = 0.0;
  fc_srv.request.des_position[0] = 0.35;
  fc_srv.request.des_position[1] = 0.0;
  fc_srv.request.des_position[2] = 0.3;
  fc_srv.request.des_normal[0] = 0.0;
  fc_srv.request.des_normal[1] = 1.0;
  fc_srv.request.des_normal[2] = 0.0;

  if (solve_fully_constrained_ik_client.call(fc_srv))
  {
	printf("Solve Fully Constrained IK:\n");
	for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
        printf("%f\t", fc_srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", fc_srv.response.feasible,
             fc_srv.response.arm_to_front, fc_srv.response.arm_bended_up, fc_srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_fully_constrained_ik");
    return 1;
  }

  return 0;
}
