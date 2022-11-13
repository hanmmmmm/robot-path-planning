# Robot Path Planning
This is a collection of my implementation of robotic 2D path planning algorithms in c++. 

This repo shows each algo individually. <br>
For a more complete navigation module, please check my this repo <a href="https://github.com/hanmmmmm/navigation_package_V1" > navigation_package_V1 </a> . 


Using only C++ standard libraries for algorithms, and OpenCV for visulization. 
</br></br>
Methods in progress:
- [ ] LPA*
- [+] Reeds-shepp (code done, need to make GIF)
- [ ] Potential field

</br>


# to-do
update some of the gifs, since they were recorded with a very early version of code, which may casue non-optimal solution.

(The code has updated, but gifs need re-recording)

----
# Demos

Each of the demo contains 5 samples played in sequence.

They are using tasks with identical start/goal. 

----
### Breadth First Searching
<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/BFS/bfs.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/BFS/bfs.gif" alt="BFS showcase gif" title="BFS search" width="500"/>
</a>

----
### Depth First Searching
<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/DFS/dfs.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/DFS/dfs.gif" alt="BFS showcase gif" title="DFS search" width="500"/>
</a>

----
### Dijkstra's 
<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/Dijkstra/dijkstra.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/Dijkstra/dijkstra.gif" alt="BFS showcase gif" title="Dijkstra search" width="500"/>
</a>

----
### D-Star

After the first path is found, the robot starts moveing along the path. </br>

The path is shown as orange line. </br>

Some obstacles are added into the map while the robot is moving. </br>

When the next step is blocked, it stops and looks for a new path to the goal. </br>

The green line is the path during replanning; the new valid path is orange line. </br>

<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/D_star/d_star.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/D_star/d_star.gif" alt="dstar showcase gif" title="D-Star search" width="500"/>
</a>


----
### A-Star

This sample is using Manhattan distance as h-cost. <br/>

<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/A_star/astar.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/A_star/astar.gif" alt="BFS showcase gif" title="A-Star search" width="500"/>
</a>

----
### Hybrid A-Star

This sample is using Euclidean distance as the primary source of h-cost. <br/>
Then modify the h-cost based on the motion direction, steer, and priximity to obstacle. </br>
The g-cost is the same as A-star. </br>

<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/hybrid_A_star/hybrid_a_star.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/hybrid_A_star/hybrid_a_star.gif" alt="Hybrid A-Star showcase gif" title="BFS search" width="500"/>
</a>

----
### Dubins curve

This sample assume that the motion model contains only forward motion. <br/>
This algo is based on the paper 
<a id="Dubins" href="https://cpb-us-e2.wpmucdn.com/faculty.sites.uci.edu/dist/e/700/files/2014/04/Dubins_Set_Robotics_2001.pdf">
    "Classification of the Dubins set"
</a>
 by Andrei M. Shkel, Vladimir Lumelsky. </br>

The types of curves being tried are listed at the top left corner of gif; and their costs are printed as well.

Those 2 black shart lines are the current pose and target pose.

<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/Dubins_curve/dubins.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/Dubins_curve/dubins.gif" alt="Dubins showcase gif" title="Dubins search" width="500"/>
</a>

----
### Probabilistic RoadMap
<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/PRM/prm.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/PRM/prm.gif" alt="PRM showcase gif" title="BFS search" width="500"/>
</a>

----
### RRT
<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/RRT/rrt.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/RRT/rrt.gif" alt="RRT showcase gif" title="RRT search" width="500"/>
</a>

----
### RRT-star

Max sample number: 1000  </br>
The final path is much smoother than RRT above. </br>

<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/RRT_star/rrt_star.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/RRT_star/rrt_star.gif" alt="RRTstar showcase gif" title="RRTstar search" width="500"/>
</a>












