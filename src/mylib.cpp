#include "mylib.h"
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <iostream>

void april_test() {
    // Initialize the AprilTag detector
    apriltag_family_t* tf = tag36h11_create();
    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // Create a test image
    image_u8_t* im = image_u8_create(640, 480);

    // Detect AprilTags in the test image
    zarray_t* detections = apriltag_detector_detect(td, im);

    // Print the number of detected AprilTags
    std::cout << "Detected " << zarray_size(detections) << " AprilTags" << std::endl;

    // Clean up
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    image_u8_destroy(im);
    apriltag_detections_destroy(detections);

}
