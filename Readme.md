# StrandbeestRobot.jl

[![Build Status](https://travis-ci.org/rdeits/StrandbeestRobot.jl.svg?branch=master)](https://travis-ci.org/rdeits/StrandbeestRobot.jl)
[![codecov.io](https://codecov.io/github/rdeits/StrandbeestRobot.jl/coverage.svg?branch=master)](https://codecov.io/github/rdeits/StrandbeestRobot.jl?branch=master)

A 12-legged robot model, inspired by Theo Jansens's [Strandbeest](http://www.strandbeest.com/), implemented in Julia using [RigidBodyDynamics.jl](https://github.com/JuliaRobotics/RigidBodyDynamics.jl) and [RigidBodySim.jl](https://github.com/JuliaRobotics/RigidBodySim.jl).

Included: 

* `data/Strandbeest.urdf`: the [URDF](http://wiki.ros.org/urdf/Tutorials) file with the kinematic and inertial parameters of the robot. Originally developed as part of the [Drake](http://drake.mit.edu/) project. 
* `notebooks/`: Jupyter notebooks demonstrating the construction and simulation of the robot in Julia
* `src/StrandbeestRobot.jl`: the `StrandbeestRobot` Julia module, containing some helper functions for working with this particular mechanism. 
