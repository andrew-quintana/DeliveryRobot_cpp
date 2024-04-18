# DeliveryRobot

## Summary
This self-parking robot project is a blend of academic expertise and genuine enthusiasm, focusing on implementing AI and algorithmic methods in a practical setting. The main goal of the project is to equip a robot with the capability to autonomously navigate and locate parking spots within a room, despite initially having limited understanding of its pose, movement, and surroundings.
I purposefully chose a robot with limited abilities in the belief that limitations can inspire creativity and innovation. Enabling me to thrive in creating high-level system designs and exploring the intricate details of the underlying design elements, I am gaining so much from this effort.

## Extent of Work
Embarking on the self-parking robot project, I delved into the creative and technical requirements. With dedication and enthusiasm, I am delving into the complexities of system design and architecture, autonomous robotics, and algorithmic decision-making for this project, which posed not only a technological challenge but also a journey of personal growth.
Conceptualization: Defining the project scope, objectives, and requirements.
- Design: Creating high-level system architectures and detailed design specifications.
- Implementation: Writing code, integrating hardware components, and configuring software systems.
- Testing and Iteration: Conducting comprehensive testing procedures, identifying areas for improvement, and iterating on the design as necessary.
- Documentation: Documenting the development process, codebase, and project outcomes for future reference.

## Technologies Used

**Programming Languages and Development Environment:**
- C++: Leveraged for the core implementation of algorithms and system logic.
- Python: Utilized for initial sample generation for testing the individual components off the hardware.
- Google Test (gtest): Employed for creating and executing unit tests to ensure code reliability and functionality.
  
**Hardware Components:**
- NVIDIA Jetson Nano: Utilized as the controller for the robot, providing computational power for real-time processing and decision-making.
- Camera: An 8MP HD resolution, 160° FOV wide-angle camera (IMX219 sensor) utilized for capturing images and detecting AprilTags.
- Differential 2WD with TT Motors: Configured as the driving setup for the robot's locomotion and maneuverability.

**Libraries and Frameworks:**
- Robot Operating System (ROS): Integrated for communication between different software modules and for managing the robot's behavior.
- Eigen: Used for linear algebra operations and transformations.
- AprilTag: Implemented for detecting and recognizing AprilTags (QR Codes) for localization and pose estimation.
- OpenCV: Employed for image processing tasks such as camera calibration, object detection, and map plotting.
- BoostGraph: Utilized for graph-based algorithms and operations.

**Simulation and Autonomy:**
- A* with Beam Search: Employed for path planning and navigation, optimizing the robot's trajectory towards parking spots.
- Online SLAM (Simultaneous Localization and Mapping): Utilized for real-time mapping of the environment and localization of the robot within it.
- Pose Estimation: Achieved through AprilTag homographies, enabling accurate determination of the robot's pose relative to detected tags.
- Computational Geometry: Integrated for building autonomy into the system, enabling the robot to make informed decisions based on geometric properties of its surroundings.
- Custom Map Plotter: Developed using OpenCV for creating simulated environments and testing scenarios.

## Challenges and Solutions

**Managing Libraries and Project Structure:**
- Challenge: The complexity of managing multiple libraries and structuring the project with CMake presented a steep learning curve.
- Solution: Through diligent research, experimentation, and seeking guidance from online resources and documentation, I gradually gained proficiency in navigating CMake and effectively managing dependencies.

**Camera Accuracy and Distance Limitations:**
- Challenge: The camera's accuracy was compromised at further distances, posing challenges for reliable detection and localization of AprilTags.
- Solution: To mitigate this challenge, I implemented limitations on the accepted ranges, including the angle in the field of view, and measured distances for the desired accuracy.

**Lack of Motor Encoder Feedback:**
- Challenge: The inability to track the distance that the motors had traveled in the absence of motor encoders or alternative hardware feedback mechanisms. Drift and bias in wheel movements also posed difficulties in getting the desired moves.
- Solution: Despite the absence of motor encoders or full wheel control, I devised creative solutions such as dead reckoning algorithms and computer vision-based pose estimation.

**Lack of Closed-Loop System:**
- Challenge: The absence of a closed-loop control system hindered real-time adjustments and responsiveness to dynamic environmental changes.
- Solution: To compensate for the lack of a closed-loop system, I implemented robust open-loop control strategies and integrated adaptive algorithms to dynamically adapt to varying conditions.

## Current State of the Project

**Progress Made:**
- Path Planning: Successfully developed and implemented algorithms for path planning, including A* with Beam Search, to optimize the robot's trajectory towards parking spots.
- Localization and Mapping: Implemented Online SLAM techniques for real-time mapping of the environment and pose estimation using AprilTag homographies.
- Sensor Development: Designed and integrated sensor systems for detecting AprilTags and gathering environmental data.

**Remaining Steps:**
- Integration: The next phase of the project involves integrating the individual components—path planning, localization and mapping, and sensor systems—into a cohesive and functional system.
- Hardware Component and System Level Testing: Once integrated, comprehensive testing and validation procedures will be conducted to ensure the reliability, accuracy, and robustness of the system in diverse scenarios.
- Refinement and Optimization: Iterative refinement and optimization of algorithms, control strategies, and hardware configurations will be undertaken to enhance the performance and efficiency of the self-parking robot.
- Documentation and Reporting: Throughout the project, meticulous documentation of the development process, codebase, and experimental results will be maintained to facilitate knowledge sharing and future reference.

**Future Directions:**
- PID Motor Control: Implement PID control algorithms to enhance motor control precision and stability, mitigating issues related to wheel drifting and bias.
- Randomized Optimization for Parameter Tuning: Explore randomized optimization techniques, such as genetic algorithms or simulated annealing, for tuning algorithm parameters and system configurations. This approach will enable automated parameter optimization and refinement, leading to improved performance and adaptability of the self-parking robot.
- Voronoi-Diagram Based Path Planning: Transition from Beam Search to a Voronoi-diagram based path planning approach, leveraging a path network derived from the Voronoi diagram to optimize trajectory planning and navigation efficiency.
