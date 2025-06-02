#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <string>

class ImageMorpher {
public:
    ImageMorpher();
    ~ImageMorpher();

    bool initialize(const std::string& sourcePath, const std::string& targetPath);

    // Point management
    void addCorrespondingPoints(const cv::Point2f& sourcePoint, const cv::Point2f& targetPoint);
    void removeLastPointPair();
    void clearAllPoints();

    // Save/load points
    bool savePointsToFile(const std::string& filename);
    bool loadPointsFromFile(const std::string& filename);

    // Triangulation
    bool calculateTriangulation();
    cv::Mat visualizeTriangulation(bool isSource);

    cv::Mat morphImages(float alpha);
    std::vector<cv::Mat> generateMorphSequence(int numFrames);

    // Getters
    const cv::Mat& getSourceImage() { return sourceImg; }
    const cv::Mat& getTargetImage() { return targetImg; }
    const std::vector<cv::Point2f>& getSourcePoints() { return sourcePoints; }
    const std::vector<cv::Point2f>& getTargetPoints() { return targetPoints; }
    bool hasTriangulation() { return !triangles.empty(); }

private:
    std::vector<cv::Point2f> calculateIntermediatePointsLinear(float alpha);

    void warpTriangle(const cv::Mat& srcImg, cv::Mat& dstImg,
        const std::vector<cv::Point2f>& srcTri,
        const std::vector<cv::Point2f>& dstTri);

    std::vector<cv::Point2f> getAllSourcePoints();
    std::vector<cv::Point2f> getAllTargetPoints();

    cv::Mat sourceImg;
    cv::Mat targetImg;
    std::vector<cv::Point2f> sourcePoints;
    std::vector<cv::Point2f> targetPoints;
    std::vector<std::vector<int>> triangles;

    static const int MAX_IMAGE_WIDTH;
};