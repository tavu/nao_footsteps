# nao_footsteps

Nao_footsteps is a navigation planner for humanoid robots based on footstep_planner.<br/>
Currently it is only tested on Nao robot. Nao_footsteps creates a graph representation of the feasible steps and applies graph search algorithms implemented in sbpl libraries(https://github.com/sbpl/sbpl).<br/>
This is ideal for Nao robots that has a separate control unit.<br/>
The purpose of the planner is to incorporates a faster heuristic and allow simultaneous moving and planning. <br/>
It is also contains a node for communication with the control unit.

## Structure
Nao_footsteps is divided to two different nodes. Footstep_planner and Footstep_handler.<br/>
* <b>Footstep_planner</b> is assigned with the task create the graph and find the requested path.<br/>
* <b>Footstep_handler</b> takes over the communication between footstep_planner and the control unit. In addition footstep_handler triggers the replanning procedure.

## Features
* Fast heuristic based on odometry model.Different cost per step
* Simultaneous moving and planning.

## Limitations
* Only supports ADPlanner since is the only capable of simultaneous moving and planning.
* Replans after fixed amound of steps\Graph searches require a discretization of the feasible steps.

## Future work
* dynamic replanning based on map errors and localization.

## Reference
* <b>Footstep_planner:</b>  https://github.com/ROBOTIS-GIT/humanoid_navigation 
* <b>Sbpl:</b> https://github.com/sbpl/sbpl