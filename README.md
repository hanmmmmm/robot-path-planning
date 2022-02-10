# Robot Path Planning
This is a collection of my implementation of robotic 2D path planning algorithms in c++. 

Using only C++ standard libraries for algorithms, and OpenCV for visulization. 

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
### Probabilistic RoadMap
<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/PRM/prm.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/PRM/prm.gif" alt="PRM showcase gif" title="BFS search" width="500"/>
</a>

----
### RRT
<a id="search" href="https://github.com/hanmmmmm/robot-path-planning/blob/main/RRT/rrt.gif">
    <img src="https://github.com/hanmmmmm/robot-path-planning/blob/main/RRT/rrt.gif" alt="RRT showcase gif" title="RRT search" width="500"/>
</a>














