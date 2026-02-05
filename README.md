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

Install VSCode and Docker on a Linux PC.

If not using Linux, the window for displaying the turtlesim won't work without
[additional display forwarding configurations](https://wiki.ros.org/docker/Tutorials/GUI).
Alternatively, use a virtual machine like UTM or Parallels to run a Linux environment on a non-Linux host.

Open this repository in VSCode with the devcontainers extension ("Reopen in Container" command).
It will then be possible to build and run ROS 2 code directly in teh container without additional steps.

### Running

The [tasks.json](.vscode/tasks.json) file defines commands to execute in the container.

- colcon build - to recompile your changes
- run turtlesim - to launch a new instance of the turtlesim window
- run behaviortree - to execute the default behavior tree XML 

These tasks should appear on the bottom row of the IDE thanks to the `spencerwmiles.vscode-task-buttons` extension.
If not, the tasks are accessible through the Command Palette (Ctrl + Shift + P), entering "Run Tasks" and selecting one of the options. 

### Command line

It is possible to run `ros2` CLI commands from anywhere in the devcontainer terminal (`ros2 run`, `ros2 node list`, `ros2 service list`, etc).

However, any `colcon` build step should always be done from the root of the ROS 2 workspace (e.g. `cd /home/ros2/ws && colcon build`).