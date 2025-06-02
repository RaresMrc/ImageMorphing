#include "ImageMorpher.h"
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>

const int ImageMorpher::MAX_IMAGE_WIDTH = 800;

ImageMorpher::ImageMorpher() {}

ImageMorpher::~ImageMorpher() {}

bool ImageMorpher::initialize(const std::string& sourcePath, const std::string& targetPath) {
    // Load images
    sourceImg = cv::imread(sourcePath);
    targetImg = cv::imread(targetPath);

    if (sourceImg.empty() || targetImg.empty()) {
        std::cerr << "Error: Could not load images" << std::endl;
        if (sourceImg.empty()) std::cerr << "Failed to load: " << sourcePath << std::endl;
        if (targetImg.empty()) std::cerr << "Failed to load: " << targetPath << std::endl;
        return false;
    }

    // Resize images
    if (sourceImg.cols > MAX_IMAGE_WIDTH) {
        float ratio = static_cast<float>(MAX_IMAGE_WIDTH) / sourceImg.cols;
        cv::resize(sourceImg, sourceImg, cv::Size(), ratio, ratio);
    }
    if (targetImg.cols > MAX_IMAGE_WIDTH) {
        float ratio = static_cast<float>(MAX_IMAGE_WIDTH) / targetImg.cols;
        cv::resize(targetImg, targetImg, cv::Size(), ratio, ratio);
    }

    // Make sure both images are the same size
    if (sourceImg.size() != targetImg.size()) {
        std::cout << "Resizing target image to match source image dimensions" << std::endl;
        cv::resize(targetImg, targetImg, sourceImg.size());
    }

    return true;
}

void ImageMorpher::addCorrespondingPoints(const cv::Point2f& sourcePoint, const cv::Point2f& targetPoint) {
    sourcePoints.push_back(sourcePoint);
    targetPoints.push_back(targetPoint);

    // Clear triangulation
    triangles.clear();
}

void ImageMorpher::removeLastPointPair() {
    if (!sourcePoints.empty() && !targetPoints.empty()) {
        sourcePoints.pop_back();
        targetPoints.pop_back();

        triangles.clear();
    }
}

void ImageMorpher::clearAllPoints() {
    sourcePoints.clear();
    targetPoints.clear();
    triangles.clear();
}

bool ImageMorpher::savePointsToFile(const std::string& filename) {
    std::ofstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    file << sourcePoints.size() << std::endl;
    for (size_t i = 0; i < sourcePoints.size(); i++) {
        file << sourcePoints[i].x << " " << sourcePoints[i].y << " "
            << targetPoints[i].x << " " << targetPoints[i].y << std::endl;
    }

    file.close();
    std::cout << "Points saved to " << filename << std::endl;
    return true;
}

bool ImageMorpher::loadPointsFromFile(const std::string& filename) {
    std::ifstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return false;
    }

    int count;
    file >> count;

    clearAllPoints();

    float sx, sy, tx, ty;
    for (int i = 0; i < count && file.good(); i++) {
        file >> sx >> sy >> tx >> ty;
        sourcePoints.push_back(cv::Point2f(sx, sy));
        targetPoints.push_back(cv::Point2f(tx, ty));
    }

    file.close();
    std::cout << "Loaded " << sourcePoints.size() << " points from " << filename << std::endl;
    return true;
}

bool ImageMorpher::calculateTriangulation() {
    if (sourcePoints.size() < 3) {
        std::cout << "Need at least 3 points for triangulation" << std::endl;
        return false;
    }

    // Create a rect that bounds all points
    cv::Rect rect(0, 0, sourceImg.cols, sourceImg.rows);

    std::vector<cv::Point2f> points = sourcePoints;
    int origSize = (int)points.size();

    // Add the corners of the image
    points.push_back(cv::Point2f(0, 0));
    points.push_back(cv::Point2f((float)rect.width - 1, 0));
    points.push_back(cv::Point2f((float)rect.width - 1, (float)rect.height - 1));
    points.push_back(cv::Point2f(0, (float)rect.height - 1));

    // Create subdiv for Delaunay triangulation
    cv::Subdiv2D subdiv(rect);

    // Insert points into subdiv
    for (const auto& point : points) {
        subdiv.insert(point);
    }

    // Get triangles as vectors of 6 coordinates (x1,y1,x2,y2,x3,y3)
    std::vector<cv::Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);

    triangles.clear();

    for (const auto& t : triangleList) {
        cv::Point2f pt1(t[0], t[1]);
        cv::Point2f pt2(t[2], t[3]);
        cv::Point2f pt3(t[4], t[5]);

        // Only consider triangles inside the image
        if (rect.contains(pt1) && rect.contains(pt2) && rect.contains(pt3)) {
            std::vector<int> indices;

            // Convert coordinates to indices
            for (const cv::Point2f& pt : { pt1, pt2, pt3 }) {
                int found = -1;
                for (int i = 0; i < points.size(); i++) {
                    if (std::abs(pt.x - points[i].x) < 1.0 && std::abs(pt.y - points[i].y) < 1.0) {
                        found = i;
                        break;
                    }
                }
                if (found != -1) {
                    indices.push_back(found);
                }
            }

            // Only add if we found all three points
            if (indices.size() == 3) {
                triangles.push_back(indices);
            }
        }
    }

    std::cout << "Calculated " << triangles.size() << " triangles" << std::endl;
    return true;
}

cv::Mat ImageMorpher::visualizeTriangulation(bool isSource) {
    if (triangles.empty()) {
        std::cout << "No triangulation available" << std::endl;
        return cv::Mat();
    }

    cv::Mat baseImg = isSource ? sourceImg.clone() : targetImg.clone();

    std::vector<cv::Point2f> points = isSource ? getAllSourcePoints() : getAllTargetPoints();

    // Draw triangulation
    for (const auto& triangle : triangles) {
        if (triangle.size() == 3) {
            cv::line(baseImg,
                cv::Point((int)points[triangle[0]].x, (int)points[triangle[0]].y),
                cv::Point((int)points[triangle[1]].x, (int)points[triangle[1]].y),
                cv::Scalar(0, 0, 255), 1);
            cv::line(baseImg,
                cv::Point((int)points[triangle[1]].x, (int)points[triangle[1]].y),
                cv::Point((int)points[triangle[2]].x, (int)points[triangle[2]].y),
                cv::Scalar(0, 0, 255), 1);
            cv::line(baseImg,
                cv::Point((int)points[triangle[2]].x, (int)points[triangle[2]].y),
                cv::Point((int)points[triangle[0]].x, (int)points[triangle[0]].y),
                cv::Scalar(0, 0, 255), 1);
        }
    }

    return baseImg;
}

std::vector<cv::Point2f> ImageMorpher::calculateIntermediatePointsLinear(float alpha) {
    if (sourcePoints.empty() || sourcePoints.size() != targetPoints.size()) {
        return {};
    }

    std::vector<cv::Point2f> intermediatePoints;

    for (size_t i = 0; i < sourcePoints.size(); i++) {
        cv::Point2f sourcePoint = sourcePoints[i];
        cv::Point2f targetPoint = targetPoints[i];

        // Linear interpolation
        float x = (1.0f - alpha) * sourcePoint.x + alpha * targetPoint.x;
        float y = (1.0f - alpha) * sourcePoint.y + alpha * targetPoint.y;

        intermediatePoints.push_back(cv::Point2f(x, y));
    }

    // Add corner points
    intermediatePoints.push_back(cv::Point2f(0, 0));
    intermediatePoints.push_back(cv::Point2f((float)sourceImg.cols - 1, 0));
    intermediatePoints.push_back(cv::Point2f((float)sourceImg.cols - 1, (float)sourceImg.rows - 1));
    intermediatePoints.push_back(cv::Point2f(0, (float)sourceImg.rows - 1));

    return intermediatePoints;
}

std::vector<cv::Point2f> ImageMorpher::getAllSourcePoints() {
    std::vector<cv::Point2f> allPoints = sourcePoints;

    // Add corners
    allPoints.push_back(cv::Point2f(0, 0));
    allPoints.push_back(cv::Point2f((float)sourceImg.cols - 1, 0));
    allPoints.push_back(cv::Point2f((float)sourceImg.cols - 1, (float)sourceImg.rows - 1));
    allPoints.push_back(cv::Point2f(0, (float)sourceImg.rows - 1));

    return allPoints;
}

std::vector<cv::Point2f> ImageMorpher::getAllTargetPoints() {
    std::vector<cv::Point2f> allPoints = targetPoints;

    // Add corners
    allPoints.push_back(cv::Point2f(0, 0));
    allPoints.push_back(cv::Point2f((float)targetImg.cols - 1, 0));
    allPoints.push_back(cv::Point2f((float)targetImg.cols - 1, (float)targetImg.rows - 1));
    allPoints.push_back(cv::Point2f(0, (float)targetImg.rows - 1));

    return allPoints;
}

// Morphing
cv::Mat ImageMorpher::morphImages(float alpha) {
    if (triangles.empty()) {
        calculateTriangulation();
    }

    if (triangles.empty() || sourcePoints.empty()) {
        return cv::Mat();
    }

    // Calculate intermediate points
    std::vector<cv::Point2f> morphPoints;
    morphPoints = calculateIntermediatePointsLinear(alpha);

    // Get all source and target points
    std::vector<cv::Point2f> allSourcePoints = getAllSourcePoints();
    std::vector<cv::Point2f> allTargetPoints = getAllTargetPoints();

    cv::Mat warpedSrc = cv::Mat::zeros(sourceImg.size(), CV_8UC3);
    cv::Mat warpedDst = cv::Mat::zeros(targetImg.size(), CV_8UC3);

    for (size_t i = 0; i < triangles.size(); i++) {
        const std::vector<int>& triangle = triangles[i];

        if (triangle.size() == 3) {
            std::vector<cv::Point2f> srcTri, dstTri, morphTri;

            for (int j = 0; j < 3; j++) {
                srcTri.push_back(allSourcePoints[triangle[j]]);
                dstTri.push_back(allTargetPoints[triangle[j]]);
                morphTri.push_back(morphPoints[triangle[j]]);
            }

            warpTriangle(sourceImg, warpedSrc, srcTri, morphTri);
            warpTriangle(targetImg, warpedDst, dstTri, morphTri);
        }
    }

    // Apply linear blending
    cv::Mat morphedImg;
    cv::addWeighted(warpedSrc, 1.0 - alpha, warpedDst, alpha, 0.0, morphedImg);

    return morphedImg;
}

std::vector<cv::Mat> ImageMorpher::generateMorphSequence(int numFrames) {
    std::vector<cv::Mat> sequence;
    sequence.reserve(numFrames + 1);

    // Generate all frames
    float alpha = 0.0f;
    float alphaIncrement = 1.0f / numFrames;
    for (int i = 0; i <= numFrames; i++) {
        cv::Mat frame = morphImages(alpha);
        if (!frame.empty()) {
            sequence.push_back(frame.clone());
        }
        alpha += alphaIncrement;
    }

    return sequence;
}

// Triangle warping
void ImageMorpher::warpTriangle(const cv::Mat& srcImg, cv::Mat& dstImg,
    const std::vector<cv::Point2f>& srcTri,
    const std::vector<cv::Point2f>& dstTri) {

    // Get bounding boxes
    cv::Rect srcRect = cv::boundingRect(srcTri);
    cv::Rect dstRect = cv::boundingRect(dstTri);

    // Clamp to image boundaries
    srcRect &= cv::Rect(0, 0, srcImg.cols, srcImg.rows);
    dstRect &= cv::Rect(0, 0, dstImg.cols, dstImg.rows);

    // Rectangle was outside image bounds
    if (srcRect.area() == 0 || dstRect.area() == 0) return;

    std::vector<cv::Point2f> srcTriOffset, dstTriOffset;
    std::vector<cv::Point> dstTriOffsetInt;

    for (int i = 0; i < 3; i++) {
        srcTriOffset.push_back(cv::Point2f(srcTri[i].x - srcRect.x, srcTri[i].y - srcRect.y));
        dstTriOffset.push_back(cv::Point2f(dstTri[i].x - dstRect.x, dstTri[i].y - dstRect.y));
        dstTriOffsetInt.push_back(cv::Point(cvRound(dstTriOffset[i].x), cvRound(dstTriOffset[i].y)));
    }

    // Calculate affine transform
    cv::Mat warpMat = cv::getAffineTransform(srcTriOffset, dstTriOffset);

    // Create triangle mask
    cv::Mat mask = cv::Mat::zeros(dstRect.height, dstRect.width, CV_8UC1);
    cv::fillConvexPoly(mask, dstTriOffsetInt, cv::Scalar(255));

    // Warp the source region, srcImg(srcRect) is a view into the source image
    cv::Mat warpedSrc;
    cv::warpAffine(srcImg(srcRect), warpedSrc, warpMat, dstRect.size(),
        cv::INTER_LINEAR, cv::BORDER_REFLECT_101);

    // Copy to destination with mask
    warpedSrc.copyTo(dstImg(dstRect), mask);
}