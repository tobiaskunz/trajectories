/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <Eigen/Core>
#include "Trajectory.h"
#include "Path.h"
#include <trajectory_smoothing/GetSmoothTraj.h>
using namespace std;
using namespace Eigen;

bool srv_call(trajectory_smoothing::GetSmoothTraj::Request &req, trajectory_smoothing::GetSmoothTraj::Response &res)
{
  // store the joint angle waypoints:
  list<VectorXd> waypoints;
  int dof=req.init_traj.points[0].positions.size();
  for(int i=0;i<req.init_traj.points.size();++i)
  {
    VectorXd waypoint(dof);
    for(int j=0;j<dof;++j)
    {
      waypoint[j]=req.init_traj.points[i].positions[j];
    }
    waypoints.push_back(waypoint);
  }

  // store the acceleration bounds:
  
  VectorXd maxAcceleration(dof); 
  VectorXd maxVelocity(dof);
  for(int i=0;i<dof;++i)
  {
    maxAcceleration[i]=req.max_acc[i];
    maxVelocity[i]=req.max_vel[i];
  }

  double max_deviation=req.max_dev;
  Trajectory trajectory(Path(waypoints, max_deviation), maxVelocity, maxAcceleration);
  trajectory.outputPhasePlaneTrajectory();

  // create output trajectory:
  trajectory_msgs::JointTrajectory out_traj;
  out_traj.joint_names=req.init_traj.joint_names;
  double dt=req.dt;
  if(trajectory.isValid())
  {
    double duration = trajectory.getDuration();
    res.duration=duration;
    cout << "Trajectory duration: " << duration << " s" << endl << endl;  
    for(double t = 0.0; t < duration; t += dt)
    {
      // storing positions:
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.positions.resize(dof);
      pt.velocities.resize(dof);
      pt.accelerations.resize(dof);
      for(int j=0;j<dof;++j)
      {
        pt.positions[j]=trajectory.getPosition(t)[j];
        pt.velocities[j]=trajectory.getVelocity(t)[j];
        // using finite differencing to get acceleration:
        if(t==0.0)
        {
          pt.accelerations[j]=(trajectory.getVelocity(t+dt)[j]-trajectory.getVelocity(t)[j])/dt;
        }
        else
        {
          pt.accelerations[j]=(trajectory.getVelocity(t)[j]-trajectory.getVelocity(t-dt)[j])/dt;
        }
      }
      out_traj.points.push_back(pt);
    }
    res.success=true;
  }
  else
  {
    cout << "Trajectory generation failed." << endl;
    res.success=false;
  }
  res.smooth_traj=out_traj;
 
  
  return true;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"trajectory_smoothing_server");
  ros::NodeHandle n;
  ros::ServiceServer service=n.advertiseService("/get_smooth_trajectory",srv_call);
  ros::spin();
  return 1;
}


