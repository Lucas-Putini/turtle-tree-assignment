# Turtle Tree Tutorial

This repository contains resources to learn about ROS 2 and Behavior Trees with the help of turtlesim.

## Turtlesim

The turtlesim package provides a highly simple simulation playground to interact with ROS topics, services and actions.

A turtle swims in a blue ocean and traces a path behind it. But, it won't move or do anything without explicit instruction!

This makes it a great interactive resource for learning about ROS 2 interfaces, such as with the following tutorial:
https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html

You can use this [devcontainer](#devcontainer-configuration) and its terminals as a ROS 2 workspace for following the tutorial above.

## Behavior Trees

Behavior trees are a way of defining complex and intelligent behavior in a flexible and controllabel way,
used in video games, robotics and AI.

See [behaviortree.dev](https://behaviortree.dev/docs/intro) for information and introductory tutorial.

This repository builds a behavior tree executor in [src/turtle_tree_tutorial/src/behavior_tree.cpp](./src/turtle_tree_tutorial/src/behavior_tree.cpp)
that interprets and executes XML trees defined in the [src/turtle_tree_tutorial/behavior_trees/](./src/turtle_tree_tutorial/behavior_trees) directory.

Any custom tree nodes must be registered in the behavior_tree.cpp implementation. The tree can then be executed with the "Run behavior tree" task
(see the [Running](#running) section).

## Devcontainer configuration

Install [VSCode](https://code.visualstudio.com/download) and [Docker](https://www.docker.com/get-started) on a Linux or Mac PC.

The default configuration of this repository is for a Linux host machine.
Open this repository in VSCode with the devcontainers extension ("Reopen in Container" command).
It will then be possible to build and run ROS 2 code directly in the container without additional steps.

#### Display configuration on Mac

On Mac, the window for displaying the turtlesim won't work without additional display forwarding configuration.

Follow the steps in [this guide](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088), then make the following local changes to the devcontainer configuration in [.devcontainer/devcontainer.json](./.devcontainer/devcontainer.json):

- Under `containerEnv`, comment out the default line `"DISPLAY": "${env:DISPLAY}",` and instead use `"DISPLAY": "host.docker.internal:0"`. 

## Intended Usage

1. Build ROS 2 workspace (if files have changed)
2. Run turtlesim
3. Run behavior tree

Modify the tree XML and any node implementations and re-run the respective steps.

### The behavior tree

The [behavior_tree.cpp](./src/turtle_tree_tutorial/src/behavior_tree.cpp) executable registers nodes and instantiates a tree from an XML file.
This file is the [example.xml](./src/turtle_tree_tutorial/behavior_trees/example.xml) tree by default. Running the executable with a command line argument
will look for a file in the [behavior_trees](./src/turtle_tree_tutorial/behavior_trees) directory. For example,
`ros2 run turtle_tree_tutorial behavior_tree my_tree.xml` will look for `src/turtle_tree_tutorial/behavior_trees/my_tree.xml`.

### Running tasks

The [tasks.json](.vscode/tasks.json) file defines commands to execute in the container.

- Build ROS 2 workspace - to recompile your changes
- Run turtlesim - to launch a new instance of the turtlesim window
- Run behavior tree - to execute the default behavior tree XML 

These tasks should appear on the bottom row of the IDE thanks to the `spencerwmiles.vscode-task-buttons` extension.
If not, the tasks are accessible through the Command Palette (Ctrl + Shift + P), entering "Run Tasks" and selecting one of the options. 

### Running from command line

It is possible to run `ros2` CLI commands from anywhere in the devcontainer terminal (`ros2 run`, `ros2 node list`, `ros2 service list`, etc).
However, any `colcon` build step should always be done from the root of the ROS 2 workspace (e.g. `cd /home/ros2/ws && colcon build`).

For example, the "Run turtlesim" task can be invoked manually with `ros2 run turtlesim turtlesim_node`. After starting the turtlesim node,
try running `ros2 node list` and then `ros2 node info /turtlesim`. You should get the following output:

```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

### Available Tree Nodes

This sample repository implements a few tree nodes to interact with the turtle via ROS 2 services

- ResetTurtle
- SpawnTurtle
- CommandTurtleSwim
- MoveTurtleRelative
- MoveTurtleAbsolute

See [turtle_interfaces.hpp](./src/turtle_tree_tutorial/include/turtle_tree_tutorial/turtle_interfaces.hpp)
for more details on the behavior and input / output ports.

Additionally, [TalkerActionNode](./src/turtle_tree_tutorial/include/turtle_tree_tutorial/TalkerActionNode.hpp) is a minimal example of a core (ROS-agnostic) BT Node.

[PatchedDelay](./src/turtle_tree_tutorial/include/turtle_tree_tutorial/delay_node.hpp) is a re-implementation of the "built-in" delay node decorator that fixes a bug (https://github.com/BehaviorTree/BehaviorTree.CPP/issues/1096)
