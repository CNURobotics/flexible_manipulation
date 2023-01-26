# Flexible Manipulation
===================

Flexible Manipulation is a collection of ROS [MoveIt!] capabilities that
interface with generic [FlexBE] compatible state implementations.  

These allow a user to graphically chain together states in a state machine to
handle basic manipulation tasks.

This allows for supervisory and sliding autonomy within the MoveIt! manipulation framework,
and better control over contingency responses and recovery behaviors.

The system is generic in the sense that it only depends on generic ROS packages,
such as *MoveIt!* and *FlexBE*, and generic ROS topics, and is not specific to any
particular robot setup.


Install
-------

A complete demonstration system is provided as part of the CHRISLab [Kinova Flexible Manipulation] demonstration.

Follow setup and operation directions there for an integrated demonstration.  

The Flexible Manipulation system has been tested using the latest version of ROS Kinetic. You
should first follow the [ROS Install Guide] and get that set up before proceeding.

To use this system with your robot, follow the standard setup instructions for MoveIt! and FlexBE, and then
add this repo to your Catkin workspace.


License
-------

Copyright (c) 2018
Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
Christopher Newport University

Portions of this code is based on code from
Copyright (c) 2015 Team ViGIR (https://github.com/team-vigir/)
Copyright (c) 2015 Spyros Maniatopoulos and Philipp Schillinger
                    (https://github.com/FlexBE/generic_flexbe_states)

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	  1. Redistributions of source code must retain the above copyright notice,
	     this list of conditions and the following disclaimer.

	  2. Redistributions in binary form must reproduce the above copyright
	     notice, this list of conditions and the following disclaimer in the
	     documentation and/or other materials provided with the distribution.

	  3. Neither the name of the copyright holder nor the names of its
	     contributors may be used to endorse or promote products derived from
	     this software without specific prior written permission.

	     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
	     WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	     POSSIBILITY OF SUCH DAMAGE.

[FlexBE]: https://flexbe.github.io
[MoveIt!]: http://moveit.ros.org
[ROS Install Guide]: http://wiki.ros.org/kinetic/Installation
[Kinova Flexible Manipulation]: https://github.com/CNURobotics/chris_kinova_flexible_manipulation
