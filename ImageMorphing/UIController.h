#pragma once

#include "ImageMorpher.h"
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

class UIController {
public:
    UIController();
    ~UIController();

    bool initialize(const std::string& sourcePath, const std::string& targetPath);
    void run();
    void printInstructions();

private:
    struct MouseCallbackData {
        UIController* controller;
        bool isSourceWindow;
    };

    // Mouse callback function
    static void onMouseWrapper(int event, int x, int y, int flags, void* userdata);
    void onMouse(int event, int x, int y, int flags, bool isSourceWindow);

    // UI update
    void updateDisplays();
    void redrawPoints();
    void showTriangulation();

    void showAnimatedPreview();
    void generateGIF();
    void deleteGeneratedFiles();

    // Fields
    std::unique_ptr<ImageMorpher> morpher;
    cv::Mat sourceDisplay;
    cv::Mat targetDisplay;
    bool selectingSource;
    int currentPointIndex;
    cv::Point2f tempSourcePoint;

    // Window names
    static const std::string SOURCE_WINDOW;
    static const std::string TARGET_WINDOW;
};