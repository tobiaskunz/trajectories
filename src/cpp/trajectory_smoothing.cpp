#include <nanobind/nanobind.h>
#include <Eigen/Core>
#include "Trajectory.h"
#include "Path.h"
#include <nanobind/ndarray.h>
#include <nanobind/eigen/dense.h>

using namespace std;
using namespace Eigen;

MatrixXd smooth_waypoints(
  const int dof, 
  const float dt,
  const float max_deviation, 
  const VectorXd max_velocity, 
  const VectorXd max_acceleration,
  MatrixXd positions)
{
  list<VectorXd> position_waypoints;
  for (int i=0; i<positions.rows(); i++)
  {
    position_waypoints.push_back(positions.row(i));
  } 
  MatrixXd out_traj = MatrixXd(1,1);

  // store the acceleration bounds 
  Trajectory trajectory(Path(position_waypoints, max_deviation), max_velocity, max_acceleration);
  // trajectory.outputPhasePlaneTrajectory();
  
  // create output trajectory:
  if(trajectory.isValid())
  {
    double duration = trajectory.getDuration();
    const int length = ceil(duration/dt);
    out_traj = MatrixXd(2*length, dof); // 
    int count = 0;

    for(double t = 0.0; t < duration; t += dt)
    {
      // storing positions:
      //MatrixXd traj_pt(2, dof);
      out_traj.row(int(t/dt )) = trajectory.getPosition(t);
      out_traj.row(int(t/dt) + (length)) = trajectory.getVelocity(t);
      //out_traj.push_back(traj_pt);
      count ++;
    }
    for(int i = count; i<length; i++)
    {
      out_traj.row(i) = trajectory.getPosition(duration);
      out_traj.row(i + length) = trajectory.getVelocity(duration);
      
    }
  }
  
  return out_traj;
 }

NB_MODULE(trajectory_smoothing_impl, m) {
    m.def("smooth", &smooth_waypoints);
}