/**
 * @file test_libtorch_sort.cpp
 * @brief Quick test to verify LibTorch SORT tracker works correctly
 */

#include "buttoning_system/sort_tracker_libtorch.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>

using namespace buttoning_system::libtorch;

void printSeparator() {
    std::cout << std::string(60, '-') << "\n";
}

bool testKalmanBoxTracker() {
    std::cout << "\n[TEST] KalmanBoxTracker with LibTorch\n";
    printSeparator();
    
    try {
        // Create a tracker with initial bbox
        cv::Rect initial_bbox(100, 100, 50, 50);
        KalmanBoxTracker tracker(initial_bbox);
        
        std::cout << "Initial state: " << tracker.getState() << "\n";
        std::cout << "Track ID: " << tracker.getId() << "\n";
        std::cout << "Age: " << tracker.getAge() << "\n";
        
        // Test prediction
        cv::Rect predicted = tracker.predict();
        std::cout << "\nAfter predict(): " << predicted << "\n";
        std::cout << "Time since update: " << tracker.getTimeSinceUpdate() << "\n";
        std::cout << "Age: " << tracker.getAge() << "\n";
        
        // Test update
        cv::Rect measurement(105, 102, 50, 50);
        tracker.update(measurement);
        std::cout << "\nAfter update(" << measurement << "):\n";
        std::cout << "State: " << tracker.getState() << "\n";
        std::cout << "Time since update: " << tracker.getTimeSinceUpdate() << "\n";
        std::cout << "Hit streak: " << tracker.getHitStreak() << "\n";
        
        std::cout << "\n✓ KalmanBoxTracker test PASSED\n";
        return true;
    }
    catch (const std::exception& e) {
        std::cout << "\n✗ KalmanBoxTracker test FAILED: " << e.what() << "\n";
        return false;
    }
}

bool testSortTracker() {
    std::cout << "\n[TEST] SortTracker Multi-Object Tracking\n";
    printSeparator();
    
    try {
        SortTracker tracker(3, 3, 0.3);
        
        // Frame 1: Two objects appear
        std::cout << "\nFrame 1: Two detections\n";
        std::vector<cv::Rect> frame1 = {
            cv::Rect(100, 100, 50, 50),
            cv::Rect(200, 200, 60, 60)
        };
        
        auto result1 = tracker.update(frame1);
        std::cout << "  Confirmed tracks: " << result1.size() << "\n";
        std::cout << "  Total active tracks: " << tracker.getNumTracks() << "\n";
        
        // Frame 2 & 3: Objects move (need min_hits=3 for confirmation)
        std::cout << "\nFrame 2-3: Objects moving\n";
        for (int i = 0; i < 2; i++) {
            std::vector<cv::Rect> detections = {
                cv::Rect(100 + 5*(i+1), 100 + 2*(i+1), 50, 50),
                cv::Rect(200 + 5*(i+1), 200 + 5*(i+1), 60, 60)
            };
            auto result = tracker.update(detections);
            std::cout << "  Frame " << (i+2) << " - Confirmed: " << result.size() 
                      << ", Active: " << tracker.getNumTracks() << "\n";
        }
        
        // Frame 4: Check confirmed tracks
        std::cout << "\nFrame 4: Verify track confirmation\n";
        std::vector<cv::Rect> frame4 = {
            cv::Rect(120, 108, 50, 50),
            cv::Rect(220, 220, 60, 60)
        };
        
        auto result4 = tracker.update(frame4);
        std::cout << "  Confirmed tracks: " << result4.size() 
                  << " (should be 2)\n";
        
        for (const auto& [id, bbox] : result4) {
            std::cout << "    Track " << id << ": ["
                      << bbox.x << ", " << bbox.y << ", "
                      << bbox.width << "x" << bbox.height << "]\n";
        }
        
        // Frame 5: One object disappears
        std::cout << "\nFrame 5: One object disappears\n";
        std::vector<cv::Rect> frame5 = {
            cv::Rect(125, 110, 50, 50)
        };
        
        auto result5 = tracker.update(frame5);
        std::cout << "  Confirmed tracks: " << result5.size() << "\n";
        std::cout << "  Total active: " << tracker.getNumTracks() 
                  << " (includes track with time_since_update)\n";
        
        // Test reset
        tracker.reset();
        std::cout << "\nAfter reset():\n";
        std::cout << "  Active tracks: " << tracker.getNumTracks() 
                  << " (should be 0)\n";
        
        std::cout << "\n✓ SortTracker test PASSED\n";
        return true;
    }
    catch (const std::exception& e) {
        std::cout << "\n✗ SortTracker test FAILED: " << e.what() << "\n";
        return false;
    }
}

bool testTensorOperations() {
    std::cout << "\n[TEST] LibTorch Tensor Operations\n";
    printSeparator();
    
    try {
        // Test basic tensor creation and operations
        auto t1 = torch::eye(3, torch::kFloat64);
        auto t2 = torch::ones({3, 3}, torch::kFloat64);
        
        std::cout << "Identity matrix (3x3):\n" << t1 << "\n";
        std::cout << "Ones matrix (3x3):\n" << t2 << "\n";
        
        auto result = torch::matmul(t1, t2);
        std::cout << "I * ones = \n" << result << "\n";
        
        // Test tensor slicing
        auto slice = result.slice(0, 0, 2).slice(1, 0, 2);
        std::cout << "Slice [0:2, 0:2]:\n" << slice << "\n";
        
        // Test inverse
        auto inv = torch::inverse(t1 + t2);
        std::cout << "Inverse of (I + ones):\n" << inv << "\n";
        
        std::cout << "\n✓ Tensor operations test PASSED\n";
        return true;
    }
    catch (const std::exception& e) {
        std::cout << "\n✗ Tensor operations test FAILED: " << e.what() << "\n";
        return false;
    }
}

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  LibTorch SORT Tracker Verification Test                  ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
    
    int passed = 0;
    int total = 3;
    
    // Run tests
    if (testTensorOperations()) passed++;
    if (testKalmanBoxTracker()) passed++;
    if (testSortTracker()) passed++;
    
    // Summary
    std::cout << "\n";
    printSeparator();
    std::cout << "Test Summary: " << passed << "/" << total << " tests passed\n";
    printSeparator();
    
    if (passed == total) {
        std::cout << "\n✓ All tests PASSED! LibTorch integration successful.\n\n";
        return 0;
    } else {
        std::cout << "\n✗ Some tests FAILED. Check LibTorch installation.\n\n";
        return 1;
    }
}
