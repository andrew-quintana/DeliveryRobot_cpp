//
// Created by Andrew Quintana on 2/6/24.
//

#include "DeliveryFSM.h"

// -------------------------------- CONSTRUCTOR/DECONSTRUCTOR ----------------------------------
DeliveryFSM::DeliveryFSM() {
	goal_ids.push(1);
	goal_ids.push(2);
}
DeliveryFSM::~DeliveryFSM() {}

// ---------------------------- GETTERS/SETTERS/UPDATERS/PRINTERS ----------------------------

MachineState DeliveryFSM::get_machine_state() { return machine_state; }

ScanState DeliveryFSM::get_scan_state() { return scan_state; }

void DeliveryFSM::set_goal_state() {
    if (measurements.count(std::to_string(goal_idx)) > 0) {
        goal_state = measurements[std::to_string(goal_idx)];
    }
    else {
        printf("WARNING: UNABLE TO SET ");
    }
    
}

const char* info_str( INFO info ) {
    switch (info) {
        case INFO::GOAL_FOUND:                  return "GOAL_FOUND";
        case INFO::GOAL_NOT_FOUND:              return "GOAL_NOT_FOUND";
        case INFO::AT_GOAL:                     return "AT_GOAL";
        case INFO::NOT_AT_GOAL:                 return "NOT_AT_GOAL";
        case INFO::ERROR:                       return "ERROR";
        case INFO::NA:                          return "NA";
        default:                                return "unknown";
    }
}

const char* machine_state_str( MachineState ms ) {
    switch (ms) {
        case MachineState::INITIALIZE:          return "INITIALIZE";
        case MachineState::SCAN:                return "SCAN";
        case MachineState::APPROACH:            return "APPROACH";
        case MachineState::PARKING:             return "PARKING";
        case MachineState::SHUTDOWN:            return "SHUTDOWN";
        case MachineState::ERROR:               return "ERROR";
        default:                                return "unknown";
    }
}

const char* scan_state_str( ScanState ss ) {
    switch (ss) {
        case ScanState::ROTATE:                 return "ROTATE";
        case ScanState::POV_MOVE:               return "POV_MOVE";
        case ScanState::ERROR:                  return "ERROR";
        default:                                return "unknown";
    }
}


void DeliveryFSM::update_log() {
    log.cycle++;

    // current states
    log.machine_current = log.machine_next;
    log.scan_current = log.scan_next;

    // decision making
    log.fsm_info = next_info;
    log.request = output;

    // next_states
    log.machine_next = get_machine_state();
    log.scan_next = get_scan_state();

    // mapping
    log.robot = robot_state;
    log.goal = goal_state;
    log.idx = goal_idx;
    log.tag1 = measurements["1"];
    log.tag2 = measurements["2"];
    log.tag8 = measurements["8"];
}

void DeliveryFSM::print_log() {
    printf("PRINTING LOG FOR CYLCE %i", log.cycle);

    // current states
    printf("\tCURRENT STATES:");
    printf("\t\tMACHINE STATE: %s", machine_state_str(log.machine_current));
    printf("\t\tSCAN STATE: %s", scan_state_str(log.scan_current));

    // decision making
    printf("\tDECISION MAKING:");
    printf("\t\tCOMMAND INFO: %s", info_str(log.fsm_info));
    printf("\t\tREQUEST: %s", log.request.c_str());

    // next states
    printf("\tNEXT STATES:");
    printf("\t\tMACHINE STATE: %s", machine_state_str(log.machine_next));
    printf("\t\tSCAN STATE: %s", scan_state_str(log.scan_next));
    
    // mapping
    printf("\tMAPPING:");
    printf("\t\tROBOT STATE: x: %f, y: %f, th: %f", log.robot[0], log.robot[1], log.robot[2]);
    printf("\t\tGOAL STATE (%f): x: %f, y: %f, th: %f", log.idx, log.robot[0], log.robot[1], log.robot[2]);
    printf("\t\tTAG1 STATE: x: %f, y: %f, th: %f", log.tag1[0], log.tag1[1], log.tag1[2]);
    printf("\t\tTAG2 STATE: x: %f, y: %f, th: %f", log.tag2[0], log.tag2[1], log.tag2[2]);
    printf("\t\tTAG8 STATE: x: %f, y: %f, th: %f", log.tag8[0], log.tag8[1], log.tag8[2]);

}

// ---------------------------------------- FSM FUNCTIONS ----------------------------------------


std::string DeliveryFSM::step_fsm( std::string img_path ) {
    // determine INFO enum for updating FSM state and output string for python interpreter

    // every single run should measure from the most recent picture
    output = "unknown -1 -1";
    measure_output = INFO::UNKNOWN;
    if (machine_state != MachineState::INITIALIZE) { measure_output = robot_measure(img_path); }
    if (measure_output == INFO::ERROR) { command_next(measure_output); return "error 0 0"; }

    switch (machine_state) {
        case MachineState::INITIALIZE:
            // initialize systems
            sensor = new AprilTagSensor(cal_DIR);
            mapping = new Mapping(&window_name);
            slam = new OnlineSLAM(3);
            nav = new Astar( beam_resolution, max_distance, heuristic_weight,
                                cost, fos, robot_radius);

            // initialize variables
            goal_state.setZero();

            next_info = INFO::NA;
            output = "na 0 0";

            break;

        case MachineState::SCAN:
            /*
            * scan has both rotate and pov move for the scalability of scan state.
            * alternative would be to make pov move the approach step and cache
            * whether or not the robot was appraoching a tag or a pov, returning to
            * the scanning step if so.
            */
            switch (get_scan_state()) {
                case ScanState::ROTATE:
                    // rotate at current position to try to identify the location of the target tag

                    rotate++;   // increment number of rotations

                    // most recent
                    if (measure_output == INFO::GOAL_FOUND) {
                        output = "wait 0 0";
                    }
                    else {
                        output = "move " +
									std::to_string(0) +
									" " +
									std::to_string(angle_interval_rad);
                    }

                    next_info = measure_output;

                    break;

                case ScanState::POV_MOVE:
                    // move toward POV generated around obstacles OR other points in the room
 
                    // check if goal seen in process of navigating POVs
                    if (measure_output == INFO::GOAL_FOUND) {
                        next_info = measure_output;
                        output = "wait 0 0";
                        break;
                    }
                    // check if at goal in process of navigating POVs
                    else if (euclidian(robot_state, goal_state) <= max_distance * fos)
                    {
                        next_info = INFO::AT_GOAL;
                        output = "wait 0 0";
                        break;
                    }
                    

                    // determine incremental movement to furthers pov
                    // TODO make this the closest then to rotate around
                    int inspect_pov = sorted_povs.top().second;
                    Astar::action next_move = nav->astar_move(
                            slam->getMap()["ROBOT"],
                            obstacles,
                            nav->get_graph()[inspect_pov].vertex_state);

                    // process info from Astar
                    if (next_move.next == INFO::AT_GOAL)
                    {
                        int num_povs = obstacle_pov_idx.size();
                        povs_visited++;
                        if (povs_visited > num_povs) { }

                        // update to next pov
                        inspect_pov = (inspect_pov == num_povs - 1) ? 0 : inspect_pov + 1;
                        output = "wait 0 0";


                    }
                    // error from astar
                    else if (next_move.next == INFO::ERROR) { output = "error 0 0"; }
                    // output should be INFO::NA to continue in POV state
                    else {
						output = "move " +
									std::to_string(next_move.distance) +
									" " +
									std::to_string(next_move.steering);
					}

                    next_info = next_move.next;
                
                    break;

            }

        case MachineState::APPROACH:

            // determine next step on path
            next_move = nav->astar_move(
                    slam->getMap()["ROBOT"],
                    obstacles,
                    goal_state);

            // error output check
            if (next_move.next == INFO::ERROR) { next_info = next_move.next; output = "error 0 0"; break; }

            // make next move on path
			output = "move " +
						std::to_string(next_move.distance) +
						" " +
						std::to_string(next_move.steering);
            next_info = next_move.next;
            break;

        case MachineState::PARKING:
            /*
            * runs three times, once to:
            *       1. move to the delivery location
            *       2. rotate to be orthogonal
            *       3. determine if the next goal has been found
            */

            // approach and reach goal
            if (!parked) {
				output = "move " +
							std::to_string(euclidian(robot_state, goal_state)) +
							" " +
							std::to_string(angle_2d(robot_state, goal_state));
                parked = true;
                next_info = INFO::NA;
                break;
            }
            
            // rotate to be orthogonal
            if (!orth) {
				output = "move " +
							std::to_string(0) +
							" " +
							std::to_string(goal_state[2] - robot_state[1]);
                orth = true;
                next_info = INFO::NA;
            }

            // update values and prepare for next steps
            parked = false;
            orth = false;
            goal_ids.pop();

            // there are no more goals
            if (goal_ids.empty()) { next_info = INFO::AT_GOAL; output = "done 0 0"; break;}

            // goals left
            goal_idx = goal_ids.front();
            
            // next goal already found
            if (measurements.count(std::to_string(goal_idx)) > 0) { next_info = INFO::GOAL_FOUND;}
            else {next_info = INFO::GOAL_NOT_FOUND;}

            output = "wait 0 0";

            break;
    }

    command_next(next_info);
    update_log();
    if (true) { print_log(); }
    return output;

}

void DeliveryFSM::command_next( INFO next ) {
    // function to determine the next state based on the current state and information produced

    std::string input;

    if (next == INFO::NA) { std::printf("\tNO COMMAND PROVIDED\t"); return; }

    switch (machine_state) {
        case MachineState::INITIALIZE:
            switch (next) {
                case INFO::NA:
                    machine_state = MachineState::SCAN;
                    break;
                case INFO::ERROR:
                    machine_state = MachineState::ERROR;
                    break;
                default:
                    std::printf("\tCOMMAND NOT RECOGNIZED IN INITIALIZE STATE\t");
                    break;
            }

        case MachineState::SCAN:
            // scan FSM
            switch (scan_state) {
                case ScanState::ROTATE:
                    switch (next) {
                        case INFO::GOAL_FOUND:
                            machine_state = MachineState::APPROACH;
                            break;
                        case INFO::GOAL_NOT_FOUND:
                            scan_state = ScanState::POV_MOVE;
                            break;
                        case INFO::AT_GOAL:
                            machine_state = MachineState::PARKING;
                            break;
                        case INFO::ERROR:
                            machine_state = MachineState::ERROR;
                            scan_state = ScanState::ERROR;
                            break;
                        default:
                            std::printf("\tCOMMAND NOT RECOGNIZED IN ROTATE STATE\t");
                            break;
                    }
                    break;

                case ScanState::POV_MOVE:
                    switch (next) {
                        case INFO::NOT_AT_GOAL:
                            scan_state = ScanState::POV_MOVE;
                            break;
                        case INFO::GOAL_FOUND:
                            machine_state = MachineState::APPROACH;
                            break;
                        case INFO::GOAL_NOT_FOUND:
                            scan_state = ScanState::ROTATE;
                            break;
                        case INFO::AT_GOAL:
                            machine_state = MachineState::PARKING;
                            break;
                        case INFO::ERROR:
                            machine_state = MachineState::ERROR;
                            scan_state = ScanState::ERROR;
                            break;
                        default:
                            std::printf("\tCOMMAND NOT RECOGNIZED IN POV_MOVE STATE\t");
                            break;
                    }
                    break;
                default:
                    std::printf("\tSTATE INVALID\t");
                    break;
            }

        case MachineState::APPROACH:
            switch (next) {
                case INFO::AT_GOAL:
                    machine_state = MachineState::PARKING;
                    break;
                case INFO::NOT_AT_GOAL:
                    break;
                case INFO::GOAL_NOT_FOUND:
                    machine_state = MachineState::SCAN;
                    scan_state = ScanState::ROTATE;
                    break;
                case INFO::ERROR:
                    machine_state = MachineState::ERROR;
                    break;
                default:
                    std::printf("\n\tCOMMAND NOT RECOGNIZED IN APPROACH STATE\t");
                    break;
            }
            break;

        case MachineState::PARKING:
            switch(next) {
                case INFO::NA:
                    machine_state = MachineState::PARKING;
                case INFO::GOAL_FOUND:
                    machine_state = MachineState::APPROACH;
                    break;
                case INFO::GOAL_NOT_FOUND:
                    machine_state = MachineState::SCAN;
                    break;
                case INFO::AT_GOAL:
                    std::cout << "Repeat process? (y/n): ";
                    std::cin >> input;
                    if (input == "y") {
                        machine_state = MachineState::INITIALIZE;
                        break;
                    } else if (input == "n") {

                    }
                    break;
                case INFO::ERROR:
                    machine_state = MachineState::ERROR;
                default:
                    std::printf("\n\tCOMMAND NOT RECOGNIZED IN APPROACH STATE\t");
                    break;
            }
    }
}


INFO DeliveryFSM::robot_measure( std::string img_path ) {

    // process image
    if (!sensor->detect(img_path, measurements)) { return INFO::ERROR; }
    // TODO add way of boxing apriltag and putting the distance on the image

    // process recognized states and clear
    slam->process_measurements(measurements);
    measurements.clear();

    // process obstacles, updating or adding locations
    process_obstacles(obstacles);

    // process last movement
    slam->process_movement(next_move.distance, next_move.steering);

    // check if the goal is a landmark in the environment
    if (measurements.count(std::to_string(goal_idx)) > 0) { return INFO::GOAL_FOUND; }
    else { return INFO::GOAL_NOT_FOUND; }

}


void DeliveryFSM::process_obstacles( std::vector<std::vector<state>>& obstacle_states_set ) {

    // iterate through each obstacle
    for (int i = 0; i < obstacle_ids.size(); ++i) {

        // generate obstacle
        state tag_state = slam->getMap()[obstacle_ids[i]];
        std::vector<state> ob = generate_obstacle(tag_state, obstacle_side_m);

        // create povs
        std::vector<state> obstacle_povs = generate_visibility_points(
                robot_radius, fos, ob
        );

        // add new obstacle entry for output
        obstacle_states_set.emplace_back();
        std::vector<state>& obstacle_states = obstacle_states_set.back();

        // add/update obstacle pov points
        for (const auto& id : obstacle_ids) {

            // clear if present to update
            obstacle_v_idx[id].clear();
            obstacle_pov_idx[id].clear();
            obstacle_povs.clear();

            // create Vertex for each obstacle and pov
            for (int j = 0; j < ob.size(); ++j) {

                // obstacle vertex
                int v_idx = nav->create_vertex();
                nav->get_graph()[v_idx].vertex_state = ob[j];
                obstacle_v_idx[id].push_back(v_idx);

                // create new vertex
                int pov_idx = nav->create_vertex();
                nav->get_graph()[pov_idx].vertex_state = ob[j];
                obstacle_v_idx[id].push_back(pov_idx);

                // add to cg interpretable obstacles
                obstacle_states.push_back(ob[j]);

                // add to distance prioritized povs
                float distance = euclidian(slam->getMap()["ROBOT"],ob[j]);
                sorted_povs.push(std::make_pair(distance, pov_idx));

            }

        }

    }

}

