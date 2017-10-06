# ROS Glucose package


This ROS package wraps a glucose SAT solver and exposes it as a set of services.

## Install

This package depends on ROS and a C++11 compatible compiler.

First, download the source code of Glucose and extract it to the `include`
folder.

        wget -qO- http://www.labri.fr/perso/lsimon/downloads/softwares/glucose-syrup-4.1.tgz | tar xvz -C include --strip-components=1

Alternatively, if you already have Glucose on your filesystem, set the `GLUCOSE_PATH` environment variable, _or_ edit the `CMakeLists.txt` file (lines 9 to 12) and set the cmake `${GLUCOSE_PATH}` variable accordingly.

Then put your package into a ROS workspace, [compile it](http://wiki.ros.org/action/fullsearch/ROS/Tutorials/BuildingPackages#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-BuildingPackages.Building_Your_Package "ROS Tutorials: Building your package"), source `devel/setup.bash` and you are ready to use it.

## Usage example

Launch ROS in terminal 1:

        roscore
    
Then launch the glucose node in terminal 2:

        rosrun ros_glucose glucose_node

### Solving one problem instance

If you only need to solve one problem instance, you can use the following
services.

In terminal 3:

        rosservice call /glucose_solver/init
        rosservice call /glucose_solver/addClause "[1,2]"
        rosservice call /glucose_solver/addClause "[-1,-2]"
        rosservice call /glucose_solver/addClause "[1]"
        rosservice call /glucose_solver/solve
        
        # Output
        # result: True
        # model: [1, -2]
    
However, if you need to add several clauses at once, there are various more convenient ways to do it:

        rosservice call /glucose_solver/addClauses "[{clause:[1,2]},{clause:[-1,-2]},{clause:[-1]}"
        rosservice call /glucose_solver/addClausesTab "[1,2,0,-1,-2,0,-1]"
        rosservice call /glucose_solver/addClausesStr "1 2 0 -1 -2 0 -1"

The 3 previous service calls have the same effect.

### Solving with assumptions

To test several assumptions in a SAT problem, it is necessary to freeze the
variables first, so that they are not eliminated during a resolution. Then,
the `solveAsmpt` service can be used.

In terminal 3:

        rosservice call /glucose_solver/init
        rosservice call /glucose_solver/addClausesStr "1 2 0 -1 -2"
        rosservice call /glucose_solver/setFrozen 1 true
        rosservice call /glucose_solver/solveAsmpts "[1]"
        
        # Output
        # result: True
        # model: [1, -2]

        rosservice call /glucose_solver/solveAsmpts "[-1]"
        
        # Output
        # result: True
        # model: [-1, 2]
        
        rosservice call /glucose_solver/solveAsmpts "[2]"
        
        # Output
        # ERROR: service [/glucose_solver/solveAsmpt] responded with an error: Assumption variable has been eliminated : 2

## Misc

The `reset` service deletes the current solver object and creates a new one.

Some error handling mechanisms have been added, mostly through exceptions
inheriting from `std::logic_error`.
