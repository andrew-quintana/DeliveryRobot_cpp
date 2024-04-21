#include "DeliveryFSM.h"

int main() {
    // looping executable function for python to call

    // create state machine instance
    //DeliveryFSM fsm = DeliveryFSM();

    // test of functionality
    std::string img_path;
    std::printf("Input Filepath: ");
    std::getline(std::cin, img_path);
    std::printf("Received Image Path!\n");
    std::printf(img_path.c_str());
	std::printf("Ending...\n");
	
	return 0;
}
