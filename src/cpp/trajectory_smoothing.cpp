#include <nanobind/nanobind.h>
#include <Eigen/Core>
#include "Trajectory.h"
#include "Path.h"
#include <nanobind/ndarray.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/tuple.h>



using namespace std;
using namespace Eigen;

std::tuple<bool, int,  double, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> smooth_waypoints(
  const int dof, 
  const double dt,
  const double max_deviation, 
  const VectorXd max_velocity, 
  const VectorXd max_acceleration,
  MatrixXd positions,
  const double input_dt,
  const int tsteps = -1)
{
  assert(max_velocity.cols() == dof);
  assert(max_acceleration.cols() == dof);
  assert(positions.cols() == dof);

  list<VectorXd> position_waypoints;
  VectorXd in_position(dof); 
  for (int i=0; i<positions.rows(); i++)
  {
    for(int j=0; j< dof; j++)
    {
      in_position[j] = positions(i,j);
    }
    position_waypoints.push_back(in_position);
  } 
  MatrixXd out_traj_pos = MatrixXd::Zero(1,1);
  MatrixXd out_traj_vel = MatrixXd::Zero(1,1);
  MatrixXd out_traj_acc = MatrixXd::Zero(1,1);
  MatrixXd out_traj_jerk = MatrixXd::Zero(1,1);
  
  // store the acceleration bounds 
  Trajectory trajectory(Path(position_waypoints, max_deviation), max_velocity, 
  max_acceleration, input_dt);
  int length = 0;
  double interpolation_dt = 0.0;

  
  // create output trajectory:
  if(trajectory.isValid())
  {
    // trajectory.outputPhasePlaneTrajectory();
    double duration = trajectory.getDuration();
    if(tsteps > 0)
    {
      length = tsteps;
      interpolation_dt = (duration) / (length-1);
    }
    else
    {
      length = ((duration+dt)/dt);
      interpolation_dt = dt;
    }
    
    out_traj_pos = MatrixXd::Zero(length, dof);
    out_traj_vel = MatrixXd::Zero(length, dof);
    out_traj_acc = MatrixXd::Zero(length, dof);
    out_traj_jerk = MatrixXd::Zero(length, dof);

    int count = 0;
    double timestep = 0;
    Eigen::VectorXd position(dof), velocity(dof), acceleration(dof);
 
    for(int t=0; t<length; t++ )
    {
      timestep = t*interpolation_dt; 
      // storing positions:
      if (timestep > duration){
        break;
      }
      trajectory.getPositionVelocityAcceleration(timestep, position, velocity, acceleration);
      out_traj_pos.row(t) = position;
      out_traj_vel.row(t) = velocity;
      //out_traj.row(t+2*length) = acceleration;
      
      
      //out_traj.row(t) = trajectory.getPosition(timestep);
      //out_traj.row(t+length) = trajectory.getVelocity(timestep);
      if(t > 0)
      {
        out_traj_acc.row(t) = (out_traj_vel.row(t) - out_traj_vel.row(t - 1)) / interpolation_dt;
        out_traj_jerk.row(t) = (out_traj_acc.row(t) - out_traj_acc.row(t- 1)) / interpolation_dt;

      }
      count ++;
    }
    
    if (count < length)
    {
  
    
    for(int i = count; i<length; i++)
    {
      out_traj_pos.row(i) = out_traj_pos.row(count-1);
      out_traj_vel.row(i) = out_traj_vel.row( count - 1);
      out_traj_acc.row(i) = out_traj_acc.row(count - 1);
      out_traj_jerk.row(i) = out_traj_jerk.row(count - 1);
      
    }
    }

  }

  std::tuple<bool, int,  double, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> result = std::make_tuple(
    trajectory.isValid(), length, interpolation_dt, out_traj_pos,
  out_traj_vel, out_traj_acc, out_traj_jerk);
  return result;

 }

bool test(){
  list<VectorXd> waypoints;
	VectorXd waypoint(3);
	waypoint << 0.0, 0.0, 0.0;
	waypoints.push_back(waypoint);
	waypoint << 0.0, 0.2, 1.0;
	waypoints.push_back(waypoint);
	waypoint << 0.0, 3.0, 0.5;
	waypoints.push_back(waypoint);
	waypoint << 1.1, 2.0, 0.0;
	waypoints.push_back(waypoint);
	waypoint << 1.0, 0.0, 0.0;
	waypoints.push_back(waypoint);
	waypoint << 0.0, 1.0, 0.0;
	waypoints.push_back(waypoint);
	waypoint << 0.0, 0.0, 1.0;
	waypoints.push_back(waypoint);

	VectorXd maxAcceleration(3);
	maxAcceleration << 1.0, 1.0, 1.0;
	VectorXd maxVelocity(3);
	maxVelocity << 1.0, 1.0, 1.0;

	Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
  return trajectory.isValid();
}

NB_MODULE(trajectory_smoothing_impl, m) {
    m.def("smooth", &smooth_waypoints);
    m.def("test", &test);
}