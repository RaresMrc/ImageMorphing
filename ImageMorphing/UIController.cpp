#include "UIController.h"
#include <iostream>

const std::string UIController::SOURCE_WINDOW = "Source Image";
const std::string UIController::TARGET_WINDOW = "Target Image";

UIController::UIController() : selectingSource(true), currentPointIndex(0) {
    morpher = std::make_unique<ImageMorpher>();
}

UIController::~UIController() {
    cv::destroyAllWindows();
}

bool UIController::initialize(const std::string& sourcePath, const std::string& targetPath) {
    if (!morpher->initialize(sourcePath, targetPath)) {
        return false;
    }

    // Create display copies
    sourceDisplay = morpher->getSourceImage().clone();
    targetDisplay = morpher->getTargetImage().clone();

    // Create windows
    cv::namedWindow(SOURCE_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(TARGET_WINDOW, cv::WINDOW_AUTOSIZE);

    static MouseCallbackData sourceData;
    static MouseCallbackData targetData;

    sourceData.controller = this;
    sourceData.isSourceWindow = true;

    targetData.controller = this;
    targetData.isSourceWindow = false;

    cv::setMouseCallback(SOURCE_WINDOW, onMouseWrapper, &sourceData);
    cv::setMouseCallback(TARGET_WINDOW, onMouseWrapper, &targetData);

    updateDisplays();

    return true;
}

void UIController::run() {
    printInstructions();

    while (true) {
        int key = cv::waitKey();

        // ESC
        if (key == 27) {
            break;
        }
        else if (key == 's' || key == 'S') {
            morpher->savePointsToFile("points.txt");
        }
        else if (key == 'l' || key == 'L') {
            if (morpher->loadPointsFromFile("points.txt")) {
                currentPointIndex = morpher->getSourcePoints().size();
                selectingSource = true;
                redrawPoints();
                updateDisplays();
            }
        }
        // Undo last point
        else if (key == 'z' || key == 'Z') {
            if (selectingSource) {
                // If we're selecting source, the last complete pair was the previous index
                if (currentPointIndex > 0) {
                    currentPointIndex--;
                    morpher->removeLastPointPair();
                    redrawPoints();
                    updateDisplays();
                }
            }
            else {
                // If we're selecting target, we've already added the source point
                if (morpher->getSourcePoints().size() > 0) {
                    morpher->removeLastPointPair();
                    redrawPoints();
                    updateDisplays();
                    selectingSource = true;
                }
            }
        }
        // Show triangulation
        else if (key == 't' || key == 'T') {
            showTriangulation();
        }
        // Animated preview
        else if (key == 'a' || key == 'A') {
            showAnimatedPreview();
        }
        // Generate GIF
        else if (key == 'g' || key == 'G') {
            generateGIF();
        }
        // Clear local generated files
        else if (key == 'x' || key == 'X') {
            deleteGeneratedFiles();
        }
        // Clear all points
        else if (key == 'c' || key == 'C') {
            morpher->clearAllPoints();
            currentPointIndex = 0;
            selectingSource = true;
            redrawPoints();
            updateDisplays();
            std::cout << "All points cleared. Select point #1 in the source image" << std::endl;
        }
    }

    cv::destroyAllWindows();
}

void UIController::printInstructions() {
    std::cout << "Image Morphing\n"
        << "Controls:\n"
        << "  ESC - Exit the program\n"
        << "  S - Save points to file\n"
        << "  L - Load points from file\n"
        << "  Z - Undo last point\n"
        << "  T - Show triangulation\n"
        << "  C - Clear all points\n"
        << "  X - Clear generated files\n"
        << "\nFast Previews:\n"
        << "  A - Animated loop preview\n"
        << "  G - Generate GIF\n"
        << "===========================================\n"
        << "Select point #1 in the source image\n";
}

void UIController::onMouseWrapper(int event, int x, int y, int flags, void* userdata) {
    MouseCallbackData* data = static_cast<MouseCallbackData*>(userdata);
    data->controller->onMouse(event, x, y, flags, data->isSourceWindow);
}

void UIController::onMouse(int event, int x, int y, int flags, bool isSourceWindow) {
    if (event != cv::EVENT_LBUTTONDOWN)
        return;

    // If the clicked window is different than the one that needs to be selected, return
    if (selectingSource != isSourceWindow)
        return;

    cv::Point2f point((float)x, (float)y);

    if (selectingSource) {
        tempSourcePoint = point;

        // Draw the point on source display
        cv::circle(sourceDisplay, point, 3, cv::Scalar(0, 255, 0), -1);
        cv::putText(sourceDisplay,
            std::to_string(currentPointIndex + 1),
            cv::Point((int)x + 5, (int)y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        cv::imshow(SOURCE_WINDOW, sourceDisplay);

        // Switch to target image
        selectingSource = false;
        std::cout << "Now select the corresponding point in the target image" << std::endl;
    }
    else {
        // We're selecting a target point, now we can add the pair
        morpher->addCorrespondingPoints(tempSourcePoint, point);

        // Draw the point on target display
        cv::circle(targetDisplay, point, 3, cv::Scalar(0, 255, 0), -1);
        cv::putText(targetDisplay,
            std::to_string(currentPointIndex + 1),
            cv::Point((int)x + 5, (int)y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        cv::imshow(TARGET_WINDOW, targetDisplay);

        currentPointIndex++;
        // Switch back to the source image
        selectingSource = true;
        std::cout << "Select point #" << (currentPointIndex + 1) << " in the source image" << std::endl;
    }
}

void UIController::updateDisplays() {
    cv::imshow(SOURCE_WINDOW, sourceDisplay);
    cv::imshow(TARGET_WINDOW, targetDisplay);
}

void UIController::redrawPoints() {
    // Reset display images
    sourceDisplay = morpher->getSourceImage().clone();
    targetDisplay = morpher->getTargetImage().clone();

    // Redraw all points
    const auto& sourcePoints = morpher->getSourcePoints();
    const auto& targetPoints = morpher->getTargetPoints();

    for (size_t i = 0; i < sourcePoints.size(); i++) {
        // Draw source point
        cv::circle(sourceDisplay, sourcePoints[i], 3, cv::Scalar(0, 255, 0), -1);
        cv::putText(sourceDisplay,
            std::to_string(i + 1),
            cv::Point((int)sourcePoints[i].x + 5, (int)sourcePoints[i].y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        // Draw target point
        cv::circle(targetDisplay, targetPoints[i], 3, cv::Scalar(0, 255, 0), -1);
        cv::putText(targetDisplay,
            std::to_string(i + 1),
            cv::Point((int)targetPoints[i].x + 5, (int)targetPoints[i].y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
}

void UIController::showTriangulation() {
    // Calculate triangulation if not already done
    if (!morpher->hasTriangulation()) {
        bool triangulationStatus = morpher->calculateTriangulation();
        if (triangulationStatus == false) return;
    }

    // Get triangulated images
    cv::Mat sourceTriangulation = morpher->visualizeTriangulation(true);
    cv::Mat targetTriangulation = morpher->visualizeTriangulation(false);

    if (!sourceTriangulation.empty() && !targetTriangulation.empty()) {
        cv::imshow("Source Triangulation", sourceTriangulation);
        cv::imshow("Target Triangulation", targetTriangulation);
    }
}

void UIController::showAnimatedPreview() {
    if (morpher->getSourcePoints().size() < 3) {
        std::cout << "Need at least 3 points to perform morphing" << std::endl;
        return;
    }

    const int numFrames = 120;
    std::string windowName = "Animated preview";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    std::cout << "Generating preview"
        << " (" << numFrames << " frames)" << std::endl;

    // Generate all frames first
    std::vector<cv::Mat> frames = morpher->generateMorphSequence(numFrames);

    if (frames.empty()) {
        std::cout << "Failed to generate frames" << std::endl;
        return;
    }

    // Play forward and backward in a loop
    while (true) {
        // Forward
        for (const auto& frame : frames) {
            cv::imshow(windowName, frame);
            if (cv::waitKey(16) >= 0) goto cleanup;
        }

        // Backward
        for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
            cv::imshow(windowName, *it);
            if (cv::waitKey(16) >= 0) goto cleanup;
        }
    }

cleanup:
    std::cout << "Animation stopped." << std::endl;
    cv::destroyWindow(windowName);
}

void UIController::generateGIF() {
    if (morpher->getSourcePoints().size() < 3) {
        std::cout << "Need at least 3 points to perform morphing" << std::endl;
        return;
    }

    const int numFrames = 60;
    std::cout << "Generating GIF with " << numFrames << " frames..." << std::endl;

    // Generate all frames
    std::vector<cv::Mat> frames = morpher->generateMorphSequence(numFrames);

    if (frames.empty()) {
        std::cout << "Failed to generate frames for GIF" << std::endl;
        return;
    }

    for (int i = frames.size() - 2; i > 0; i--) {
        frames.push_back(frames[i]);
    }

    std::cout << "Generated " << frames.size() << " frames" << std::endl;

    // Output filename
    std::string gifPath = "morph_output.gif";

    // Create temporary folder for frames
    std::string frameFolder = "temp_gif_frames";
#ifdef _WIN32
    system(("mkdir " + frameFolder + " 2>nul").c_str());
#else
    system(("mkdir -p " + frameFolder).c_str());
#endif

    std::cout << "Saving temporary frames..." << std::endl;

    // Save frames as PNG files
    for (size_t i = 0; i < frames.size(); i++) {
        std::string filename = frameFolder + "/frame_" +
            std::string(3 - std::to_string(i).length(), '0') +
            std::to_string(i) + ".png";
        cv::imwrite(filename, frames[i]);
    }

    std::cout << "Creating GIF file: " << gifPath << std::endl;

    // Create FFmpeg commands
    std::string paletteCmd, gifCmd;

#ifdef _WIN32
    paletteCmd = "ffmpeg -y -r 60 -i " + frameFolder + "\\frame_%03d.png -vf \"palettegen\" palette.png >nul 2>&1";
    gifCmd = "ffmpeg -y -r 60 -i " + frameFolder + "\\frame_%03d.png -i palette.png -lavfi \"paletteuse\" " + gifPath + " >nul 2>&1";
#else
    paletteCmd = "ffmpeg -y -r 60 -i " + frameFolder + "/frame_%03d.png -vf \"palettegen\" palette.png >/dev/null 2>&1";
    gifCmd = "ffmpeg -y -r 60 -i " + frameFolder + "/frame_%03d.png -i palette.png -lavfi \"paletteuse\" " + gifPath + " >/dev/null 2>&1";
#endif

    // Execute FFmpeg commands
    int paletteResult = system(paletteCmd.c_str());

    if (paletteResult == 0) {
        int gifResult = system(gifCmd.c_str());

        if (gifResult == 0) {
            std::cout << "\nGIF created successfully: " << gifPath << std::endl;

            // Clean up temporary files
            std::cout << "Cleaning up temporary files..." << std::endl;

#ifdef _WIN32
            system(("rmdir /s /q " + frameFolder + " >nul 2>&1").c_str());
            system("del palette.png >nul 2>&1");
#else
            system(("rm -rf " + frameFolder + " >/dev/null 2>&1").c_str());
            system("rm -f palette.png >/dev/null 2>&1");
#endif

            std::cout << "Temporary files cleaned up." << std::endl;

            std::cout << "\nShowing GIF preview (press any key to stop)" << std::endl;
            cv::namedWindow("GIF Preview", cv::WINDOW_AUTOSIZE);

            while (true) {
                for (const auto& frame : frames) {
                    cv::imshow("GIF Preview", frame);
                    if (cv::waitKey(16) >= 0) goto cleanup_gif;
                }
            }

        cleanup_gif:
            cv::destroyWindow("GIF Preview");
        }
        else {
            std::cerr << "Failed to create GIF. FFmpeg error." << std::endl;
        }

    }
}

void UIController::deleteGeneratedFiles() {
    std::cout << "Clearing generated files" << std::endl;

    std::vector<std::string> foldersToClean = {
        "temp_gif_frames"
    };

    std::vector<std::string> filesToClean = {
        "morph_output.gif",
        "palette.png"
    };

    int deletedCount = 0;

    for (const auto& folder : foldersToClean) {
#ifdef _WIN32
        std::string command = "rmdir /s /q \"" + folder + "\" 2>nul";
#else
        std::string command = "rm -rf \"" + folder + "\" 2>/dev/null";
#endif

        if (system(command.c_str()) == 0) {
            std::cout << "Deleted folder: " << folder << std::endl;
            deletedCount++;
        }
    }

    for (const auto& file : filesToClean) {
#ifdef _WIN32
        std::string command = "del \"" + file + "\" 2>nul";
#else
        std::string command = "rm -f \"" + file + "\" 2>/dev/null";
#endif

        if (system(command.c_str()) == 0) {
            std::cout << "Deleted file: " << file << std::endl;
            deletedCount++;
        }
    }

    if (deletedCount == 0) {
        std::cout << "No generated files found to clean." << std::endl;
    }
    else {
        std::cout << "Cleanup completed. Deleted " << deletedCount << " items." << std::endl;
    }
}