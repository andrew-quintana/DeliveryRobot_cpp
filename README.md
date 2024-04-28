This effort is on the back burner until a certain maturity of the other DeliveryRobot project is complete.

As of 05/28/2024, the plan is to finish these in DeliveryRobot:
1. complete basic functionality of FSM and components for navigation, slam, obstacle avoidance, ordered parking
2. constant movement
3. conversion to kalman filter-based slam
4. values in ML model for motor tuning with Apriltag sensing & slam kalman confidence as feedback (distance and weights)

Once these are completed, the roadmap will include:
1. component modularization (LiDAR mapping, epipolar mapping, encoder measurement recall, IMU recall)
2. A* redundancy planning: Astar prioritization of redundant systems and methods of navigation inspired by F.E.A.R. video game AI
