# StrandbeestRobot.jl

[![Build Status](https://travis-ci.org/rdeits/StrandbeestRobot.jl.svg?branch=master)](https://travis-ci.org/rdeits/StrandbeestRobot.jl)
[![codecov.io](https://codecov.io/github/rdeits/StrandbeestRobot.jl/coverage.svg?branch=master)](https://codecov.io/github/rdeits/StrandbeestRobot.jl?branch=master)

A 12-legged robot model, inspired by Theo Jansens's [Strandbeest](http://www.strandbeest.com/), implemented in [Julia](https://julialang.org/) using [RigidBodyDynamics.jl](https://github.com/JuliaRobotics/RigidBodyDynamics.jl) and [RigidBodySim.jl](https://github.com/JuliaRobotics/RigidBodySim.jl).

Included: 

* `data/Strandbeest.urdf`: the [URDF](http://wiki.ros.org/urdf/Tutorials) file with the kinematic and inertial parameters of the robot. Originally developed as part of the [Drake](http://drake.mit.edu/) project. 
* `notebooks/`: Jupyter notebooks demonstrating the construction and simulation of the robot in Julia
* `src/StrandbeestRobot.jl`: the `StrandbeestRobot` Julia module, containing some helper functions for working with this particular mechanism. 

# Examples

## Walking on Flat Ground

[![Strandbeest robot visualization](https://user-images.githubusercontent.com/591886/45247495-16333400-b2d6-11e8-932a-c50667d9d52d.png)](https://youtu.be/1MUHuovdOEc)

## Walking Passively Downhill

[![Strandbeest on a hill](https://user-images.githubusercontent.com/591886/45247573-82159c80-b2d6-11e8-810e-61c442d8aa10.png)](https://youtu.be/T1frKeoPa_4)
