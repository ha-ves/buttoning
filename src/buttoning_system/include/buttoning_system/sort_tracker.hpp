// SORT (Simple Online and Realtime Tracking) Tracker Implementation
// Based on: "Simple Online and Realtime Tracking" by Bewley et al., 2016
// https://arxiv.org/abs/1602.00763

#ifndef SORT_TRACKER_HPP_
#define SORT_TRACKER_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace buttoning_system {

/**
 * @brief Kalman Filter for tracking bounding boxes in image space
 * 
 * State vector: [x, y, s, r, vx, vy, vs]
 * - x, y: center position
 * - s: scale (area)
 * - r: aspect ratio (width/height)
 * - vx, vy, vs: velocities
 * 
 * Measurement vector: [x, y, s, r]
 */
class KalmanBoxTracker {
public:
    /**
     * @brief Constructor
     * @param bbox Initial bounding box [x1, y1, x2, y2]
     */
    explicit KalmanBoxTracker(const cv::Rect& bbox);
    
    /**
     * @brief Update the state with a new measurement
     * @param bbox Bounding box measurement [x1, y1, x2, y2]
     */
    void update(const cv::Rect& bbox);
    
    /**
     * @brief Predict the next state
     * @return Predicted bounding box [x1, y1, x2, y2]
     */
    cv::Rect predict();
    
    /**
     * @brief Get the current state as bounding box
     * @return Current bounding box [x1, y1, x2, y2]
     */
    cv::Rect getState() const;
    
    /**
     * @brief Get the number of frames since last update
     */
    int getTimeSinceUpdate() const { return time_since_update_; }
    
    /**
     * @brief Get the number of consecutive hits
     */
    int getHitStreak() const { return hit_streak_; }
    
    /**
     * @brief Get the age of the tracker
     */
    int getAge() const { return age_; }
    
    /**
     * @brief Get the track ID
     */
    int getId() const { return id_; }
    
private:
    /**
     * @brief Convert bounding box to state vector [x, y, s, r]
     * @param bbox Bounding box [x1, y1, x2, y2]
     * @return State vector
     */
    Eigen::Vector4d bboxToZ(const cv::Rect& bbox) const;
    
    /**
     * @brief Convert state vector to bounding box
     * @param x State vector [x, y, s, r, ...]
     * @return Bounding box [x1, y1, x2, y2]
     */
    cv::Rect xToBbox(const Eigen::VectorXd& x) const;
    
    // Kalman filter matrices
    Eigen::MatrixXd F_; // State transition matrix (7x7)
    Eigen::MatrixXd H_; // Measurement matrix (4x7)
    Eigen::MatrixXd Q_; // Process noise covariance (7x7)
    Eigen::MatrixXd R_; // Measurement noise covariance (4x4)
    Eigen::MatrixXd P_; // State covariance (7x7)
    
    // State
    Eigen::VectorXd x_; // State vector (7x1)
    
    // Tracking metadata
    int id_;
    int time_since_update_;
    int hit_streak_;
    int age_;
    
    static int count_; // Global counter for unique IDs
};

/**
 * @brief SORT Tracker - Simple Online and Realtime Tracking
 * 
 * Manages multiple object tracks using Kalman filtering and Hungarian algorithm
 * for data association based on IOU (Intersection over Union).
 */
class SortTracker {
public:
    /**
     * @brief Constructor
     * @param max_age Maximum number of frames to keep alive a track without matches
     * @param min_hits Minimum number of associated detections before track is confirmed
     * @param iou_threshold Minimum IOU for matching detections to tracks
     */
    SortTracker(int max_age = 1, int min_hits = 3, double iou_threshold = 0.3);
    
    /**
     * @brief Update tracker with new detections
     * @param detections Vector of bounding boxes [x1, y1, x2, y2]
     * @param scores Detection confidence scores (optional, for filtering)
     * @return Vector of tracked objects with IDs and bounding boxes
     */
    std::vector<std::pair<int, cv::Rect>> update(
        const std::vector<cv::Rect>& detections,
        const std::vector<float>& scores = std::vector<float>());
    
    /**
     * @brief Get current active track IDs in order of detections
     * @return Vector of track IDs corresponding to last update's detections
     */
    std::vector<int> getTrackIds() const { return last_track_ids_; }
    
    /**
     * @brief Reset the tracker (clear all tracks)
     */
    void reset();
    
    /**
     * @brief Get the number of active tracks
     */
    size_t getNumTracks() const { return trackers_.size(); }

private:
    /**
     * @brief Compute IOU (Intersection over Union) between two bounding boxes
     * @param bb_test First bounding box
     * @param bb_gt Second bounding box
     * @return IOU value [0, 1]
     */
    double iou(const cv::Rect& bb_test, const cv::Rect& bb_gt) const;
    
    /**
     * @brief Associate detections to existing trackers using Hungarian algorithm
     * @param detections Vector of detection bounding boxes
     * @param trackers Vector of tracker predictions
     * @param matched Indices of matched pairs (detection_idx, tracker_idx)
     * @param unmatched_dets Indices of unmatched detections
     * @param unmatched_trks Indices of unmatched trackers
     */
    void associateDetectionsToTrackers(
        const std::vector<cv::Rect>& detections,
        const std::vector<cv::Rect>& trackers,
        std::vector<std::pair<int, int>>& matched,
        std::vector<int>& unmatched_dets,
        std::vector<int>& unmatched_trks);
    
    /**
     * @brief Solve linear assignment problem using Hungarian algorithm
     * @param cost_matrix Cost matrix (higher cost = worse match)
     * @return Vector of matched pairs (row_idx, col_idx)
     */
    std::vector<std::pair<int, int>> hungarianAlgorithm(
        const Eigen::MatrixXd& cost_matrix);
    
    // Tracker parameters
    int max_age_;
    int min_hits_;
    double iou_threshold_;
    
    // Active trackers
    std::vector<std::unique_ptr<KalmanBoxTracker>> trackers_;
    
    // Track IDs from last update
    std::vector<int> last_track_ids_;
    
    // Frame counter
    int frame_count_;
};

} // namespace buttoning_system

#endif // SORT_TRACKER_HPP_
