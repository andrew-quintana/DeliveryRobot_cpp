#include "DeliveryFSM.h"

int main() {
    // looping executable function for python to call

    // create state machine instance
    //DeliveryFSM fsm = DeliveryFSM();

    // test of functionality
    printf("Input Filepath: ");
    std::getline(std::cin, img_path);
    printf("Received Image Path!");
    cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);
    cv::imshow("Image", image);
    cv::waitKey(0);
    cv::destroyAllWindows();

}