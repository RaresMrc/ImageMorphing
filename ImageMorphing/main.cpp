#include "UIController.h"
#include <iostream>

int main(int argc, char** argv) {
    // Check program arguments
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <source_image> <target_image>" << std::endl;
        std::cout << "Example: " << argv[0] << " source.jpg target.jpg" << std::endl;
        return -1;
    }

    UIController controller;

    // Initialize with the provided images
    if (!controller.initialize(std::string(argv[1]), std::string(argv[2]))) {
        std::cerr << "Failed to initialize application" << std::endl;
        return -1;
    }

    controller.run();

    return 0;
}