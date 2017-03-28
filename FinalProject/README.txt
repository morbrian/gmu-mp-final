Brian Moriarty / Stuart Roettger
CS689 - Planning Motions of Robots
Final Project - Spring 2012
-----------------------------------

To compile the program use the same process as for compiling homework 3.  

In a command window on Windows, cd to the folder where the file "CMakeLists" is and run: cmake -G "Visual Studio 10"

Open the project in Visual Studio and build the project.

Once the the project has been built you can run it by running: 
bin\Debug\Planner.exe <configuration file>

In the robots section of the configuration files there is an option for RRT with path smoothing.  This option only works with one robot.  The timing doesn't transfer correctly once the path is smoothed so robots can end up occupying the same space.

Below are the options that can be used while the program is running.

Keyboard Options:
esc: quit
r: modify obstacle radii
p: toggle planning start/stop
v: toggle drawing of vertices
d: debug output for robot vertices to console
t: take a single step in planning or animation
z/Z: rotate positive(z) or negative(Z) on z access
x/X: rotate positive(x) or negative(X) on x access
y/Y: rotate positive(y) or negative(Y) on y access
1..4: Adjust animation speed (1 = fast, 4 = slowest)
s: save scene (robots and obstacles) to scene.txt file


Format of Configuration files(some sample configuration files have been included):

BBox [xmin] [ymin] [xmax] [ymax]
Describes workspace bounding box

DistOneStep [num]
Distance robot travels per step.

Display [type]
Indicate 2D or 3D, possible values in set (2 3)

PriorityType [type]
Specify priority algorithm, values in set (0 1 2)
0 = Plan in order of appearance in config file.
1 = Plan in reverse order of appearance in config file.
2 = Plan using minimum average obstacle distance.

Robots
[num]
A1
A2
Indicate the number of robots, and one robot per line.
Each robot line is formatted as:
[algorithm] [x] [y] [length] [width] [orientation] [goal-x] [goal-y] [goal-radius]
Where algorithm indicate single robot planner to use from the set of (0 1 2 3)
0: RRT
1: Random
2: EST
3: RRT with Path Smoothing 

Obstacles
[num]
O1
O2
Indicates the number of obstacles, with one obstacle per line.
[x] [y] [radius]
