#include "mylib.h"
#include <iostream>
#include <fstream>

void testApriltag() {
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

void testOpencv( const std::string& DIR ) {

	const std::string inputSettingsFile = DIR + "/default.xml";
	std::cout << DIR << std::endl;
	std::cout << inputSettingsFile << std::endl;
	std::ifstream file(inputSettingsFile);
	
	if (file.is_open())
    {
        cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings

        // intrinsic parameters
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> distance_coefficients;
        fs["rotation_vectors"] >> rotation_vectors;
        fs["translation_vectors"] >> translation_vectors;
        fs["frame_size"] >> frame_size;
        fs.release();
    }
    
    std::cout << "Camera_Matrix: " << camera_matrix << std::endl;
    std::cout << "Distance_Coefficients: " << distance_coefficients << std::endl;
    std::cout << "rotation_vectors: " << rotation_vectors << std::endl;
    std::cout << "translation_vectors: " << translation_vectors << std::endl;
    std::cout << "frame_size: " << frame_size << std::endl;

}

/*
bool testEigen() {
    // Create a simple Eigen matrix
    Eigen::Matrix3d matrix;
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;

    // Perform a simple operation on the matrix
    Eigen::Matrix3d result = matrix * matrix;

    // Check if the result is as expected
    Eigen::Matrix3d expected;
    expected << 30, 36, 42,
                66, 81, 96,
                102, 126, 150;

    return (result == expected).all();
}

bool testBoost() {
    std::string input = "  Hello, World!  ";
    std::string trimmed;

    // Use Boost's trim function to remove leading and trailing whitespace
    boost::trim(input, trimmed);

    // Check if the trimmed string is as expected
    return trimmed == "Hello, World!";
}

TEST(ExampleTests, TestAddition) {
    int a = 2;
    int b = 3;
    int result = a + b;
    EXPECT_EQ(result, 5);
}
*/
