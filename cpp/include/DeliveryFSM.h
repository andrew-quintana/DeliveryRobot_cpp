//
// Created by Andrew Quintana on 2/6/24.
//

#ifndef DELIVERYFSM_H
#define DELIVERYFSM_H

#pragma once

#include "AprilTagSensor.h"
#include "Astar.h"
#include "ComputationalGeometry.h"
#include "Mapper.h"
#include "OnlineSLAM.h"
#include "Utilities.h"

#include <map>
#include <array>
#include <iostream>
#include <queue>

enum class INFO {
    GOAL_FOUND,
    GOAL_NOT_FOUND,
    AT_GOAL,
    NOT_AT_GOAL,
    ERROR,
    NA
};

enum class MachineState {
    INITIALIZE,
    SCAN,
    APPROACH,
    PARKING,
    SHUTDOWN,
    ERROR
};

enum class ScanState {
    ROTATE,
    POV_MOVE,
    ERROR
};

class DeliveryFSM : public Component {
private:

    MachineState machine_state = MachineState::INITIALIZE;

    ScanState scan_state = ScanState::ROTATE;

public:

    DeliveryFSM();
    ~DeliveryFSM();

    void command_next(INFO next = INFO::NA );

    MachineState get_machine_state();
    ScanState get_scan_state();

};

std::string step_fsm();
std::string info_str( INFO info );
std::string machine_state_str( MachineState ms );
std::string scan_state_str( ScanState ss );

#endif //JETBOTPARKING_DELIVERYFSM_H
