//
// Created by Andrew Quintana on 2/7/24.
//

#ifndef DELIVERYINTERFACE_H
#define DELIVERYINTERFACE_H

#include "DeliveryFSM.h"

#include <map>
#include <queue>

// ------------------------- AVAILABLE FOR DEBUGGING --------------------------
bool debug = true;

struct Log {
    int cycle = 0;

    // current states
    MachineState machine_current = MachineState::INITIALIZE;
    ScanState scan_current = ScanState::ROTATE;

    // decision making
    INFO fsm_info;
    std::string request;

    // next states
    MachineState machine_next;
    ScanState scan_next;

    // mapping
    Eigen::Vector3f robot;
    Eigen::Vector3f goal;
    int idx;
    Eigen::Vector3f tag1;
    Eigen::Vector3f tag2;
    Eigen::Vector3f tag8;
}


// ---------------------------- OBJECT DECLARATION ----------------------------
// state machine
DeliveryFSM fsm = *new DeliveryFSM();
MachineState machine_state = fsm.get_machine_state;
INFO next_info;
INFO measure_output;
Log log;

// apriltag
AprilTagSensor* sensor;

// slam algorithm - onlineslam
OnlineSLAM* slam;

// search algorithm - astar
Astar* nav;
state robot_state;
Astar::action next_move;
int beam_resolution = 5;
float max_distance = 0.010; // TODO H2: consider changing max move distance to the minimum scan distance
float heuristic_weight = 1;
float cost = 1;
float fos = 1.25;
float robot_radius = 0.015;


// mapper
std::string window_name = "Radar";
Mapping* mapping;

// robot AND/OR simulator
RobotSim* robot;
float velocity_m_s = 5;
float angular_velocity_rad_s = 5;


// ---------------------------- VARIABLE DECLARATION ----------------------------
int goal_idx = 0;
std::queue<int> goal_ids = {1, 2};    // TODO need method of iterating through goals and passing/updating
state goal_state;
env measurements;
std::vector<std::string> obstacle_ids = {"8"};
std::map<std::string, std::vector<int>> obstacle_v_idx;
std::map<std::string, std::vector<int>> obstacle_pov_idx;
std::vector<std::vector<state>> obstacles;
std::priority_queue<std::pair<float,int>> sorted_povs;
int povs_visited = 0;

// computational geometry
float obstacle_side_m = 0.020;

// SCAN variables
int scan_resolution = 8;    // quantity of rotations required
int rotate = 0;             // quantity of rotations executed
float angle_interval_rad = (2 * M_PI) / scan_resolution;

// PARKING variables
bool parked = false;
bool orth = false;

// ---------------------------- FUNCTION DECLARATION ----------------------------
INFO robot_measure( std::string img_path );
void set_goal_state();
void process_obstacles( std::vector<std::vector<state>>& obstacle_states_set );
Log update_log();
void print_log();

#endif //DELIVERYINTERFACE_H