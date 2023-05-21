# Trajectory Smoothing

This repository is a wrapper around Tobias Kunz and Mike Stillman's time-optimal trajectory 
generation method from the below paper:

> Paper: Kunz, Tobias, and Mike Stilman. "Time-optimal trajectory generation for path following 
with bounded acceleration and velocity." Robotics: Science and Systems VIII (2012): 1-8.

original cpp code: https://github.com/tobiaskunz/trajectories


There are two wrappers in this repository:

1. ros_1 wrapper: https://github.com/balakumar-s/trajectory_smoothing/tree/v0.1

2. python wrapper: main branch

## Installing the python wrapper

1. `pip install trajectory_smoothing @ https://github.com/balakumar-s/trajectory_smoothing.git`
2. Look at `examples/smooth_example.py` for an example.