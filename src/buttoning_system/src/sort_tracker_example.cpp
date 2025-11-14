/**
 * @file sort_tracker_example.cpp
 * @brief Example usage of the SORT (Simple Online and Realtime Tracking) tracker
 * 
 * This example demonstrates how to use the SORT tracker for multi-object tracking.
 */

#include "buttoning_system/sort_tracker.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace buttoning_system;

int main() {
    std::cout << "SORT Tracker Example\n";
    std::cout << "====================\n\n";
    
    // Create SORT tracker with parameters:
    // - max_age: 3 frames (maximum frames to keep alive a track without matches)
    // - min_hits: 3 (minimum consecutive detections before track is confirmed)
    // - iou_threshold: 0.3 (minimum IOU for matching detections to tracks)
    SortTracker tracker(3, 3, 0.3);
    
    // Simulate detections over multiple frames
    std::cout << "Frame 1: Two detections\n";
    std::vector<cv::Rect> frame1_detections = {
        cv::Rect(100, 100, 50, 50),  // Detection 1
        cv::Rect(200, 200, 60, 60)   // Detection 2
    };
    std::vector<float> frame1_scores = {0.9f, 0.85f};
    
    auto tracked1 = tracker.update(frame1_detections, frame1_scores);
    std::cout << "  Confirmed tracks: " << tracked1.size() << "\n";
    std::cout << "  Active tracks: " << tracker.getNumTracks() << "\n";
    auto track_ids1 = tracker.getTrackIds();
    std::cout << "  Track IDs: ";
    for (int id : track_ids1) {
        std::cout << id << " ";
    }
    std::cout << "\n\n";
    
    // Frame 2: Same objects moved slightly
    std::cout << "Frame 2: Same objects moved\n";
    std::vector<cv::Rect> frame2_detections = {
        cv::Rect(105, 102, 50, 50),  // Detection 1 moved
        cv::Rect(205, 205, 60, 60)   // Detection 2 moved
    };
    std::vector<float> frame2_scores = {0.88f, 0.87f};
    
    auto tracked2 = tracker.update(frame2_detections, frame2_scores);
    std::cout << "  Confirmed tracks: " << tracked2.size() << "\n";
    std::cout << "  Active tracks: " << tracker.getNumTracks() << "\n";
    auto track_ids2 = tracker.getTrackIds();
    std::cout << "  Track IDs: ";
    for (int id : track_ids2) {
        std::cout << id << " ";
    }
    std::cout << "\n\n";
    
    // Frame 3: Objects continue moving
    std::cout << "Frame 3: Objects continue moving\n";
    std::vector<cv::Rect> frame3_detections = {
        cv::Rect(110, 105, 50, 50),
        cv::Rect(210, 210, 60, 60)
    };
    std::vector<float> frame3_scores = {0.91f, 0.89f};
    
    auto tracked3 = tracker.update(frame3_detections, frame3_scores);
    std::cout << "  Confirmed tracks: " << tracked3.size() << " (should be 2 now - min_hits reached)\n";
    std::cout << "  Active tracks: " << tracker.getNumTracks() << "\n";
    for (const auto& track : tracked3) {
        std::cout << "    Track ID " << track.first << ": bbox = ["
                  << track.second.x << ", " << track.second.y << ", "
                  << track.second.width << ", " << track.second.height << "]\n";
    }
    std::cout << "\n";
    
    // Frame 4: One object disappears
    std::cout << "Frame 4: Detection 2 disappears\n";
    std::vector<cv::Rect> frame4_detections = {
        cv::Rect(115, 108, 50, 50)  // Only detection 1
    };
    std::vector<float> frame4_scores = {0.90f};
    
    auto tracked4 = tracker.update(frame4_detections, frame4_scores);
    std::cout << "  Confirmed tracks: " << tracked4.size() << "\n";
    std::cout << "  Active tracks: " << tracker.getNumTracks() << " (includes unmatched track)\n";
    auto track_ids4 = tracker.getTrackIds();
    std::cout << "  Track IDs: ";
    for (int id : track_ids4) {
        std::cout << id << " ";
    }
    std::cout << "\n\n";
    
    // Frame 5: New detection appears
    std::cout << "Frame 5: New detection appears\n";
    std::vector<cv::Rect> frame5_detections = {
        cv::Rect(120, 110, 50, 50),  // Detection 1 continues
        cv::Rect(300, 100, 40, 40)   // New detection 3
    };
    std::vector<float> frame5_scores = {0.92f, 0.86f};
    
    auto tracked5 = tracker.update(frame5_detections, frame5_scores);
    std::cout << "  Confirmed tracks: " << tracked5.size() << "\n";
    std::cout << "  Active tracks: " << tracker.getNumTracks() << "\n";
    auto track_ids5 = tracker.getTrackIds();
    std::cout << "  Track IDs: ";
    for (int id : track_ids5) {
        std::cout << id << " ";
    }
    std::cout << "\n\n";
    
    // Reset tracker
    std::cout << "Resetting tracker...\n";
    tracker.reset();
    std::cout << "  Active tracks after reset: " << tracker.getNumTracks() << "\n";
    
    std::cout << "\nExample complete!\n";
    
    return 0;
}
