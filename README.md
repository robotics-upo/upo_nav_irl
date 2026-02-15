# upo_nav_irl
A C++ implementation of the IRL algorithms RTIRL[1] and RLT[2]. These algorithms replace the MDP framework by a RRT* planner[3]. So, their goal is to learn the weights of the RRT* cost function that lead the planner to behave similarly to the demonstrations.

The C++ library has interface (planning.h) that must be implemented in order to connect the learning with the desired RRT* planner and load properly the data for learning.
In this case, the library of RRT* planners upo_rrt_planners and ROS has been used to implement the interface and perform the learning.

#### TODO
- [ ] Include other learning algorithms like Maximum Margin Planning (MMP) that uses an A* planner. 


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.


[1] Pérez-Higueras, N., Caballero, F. & Merino, L. Teaching Robot Navigation Behaviors to Optimal RRT Planners. Int J of Soc Robotics 10, 235–249 (2018). https://doi.org/10.1007/s12369-017-0448-1

[2] K. Shiarlis, J. Messias, and S. Whiteson, "Rapidly exploring learning trees", in Proceedings of the IEEE International Conference on Robotics and Automation (ICRA). Singapore, Singapore: IEEE, May 2017. [Online]. Available: http://teresaproject.eu/project/publications-new

[3] S. Karaman and E. Frazzoli, "Sampling-based algorithms for optimal motion planning", The International Journal of Robotics Research, vol. 30, no. 7, pp. 846–894, 2011.
