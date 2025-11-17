// SORT (Simple Online and Realtime Tracking) Tracker Implementation with LibTorch
// Based on: "Simple Online and Realtime Tracking" by Bewley et al., 2016
// https://arxiv.org/abs/1602.00763

#ifndef SORT_TRACKER_LIBTORCH_HPP_
#define SORT_TRACKER_LIBTORCH_HPP_

#include <vector>
#include <memory>
#include <torch/torch.h>
#include <torchvision/vision.h>

namespace buttoning_system {
namespace libtorch {

/**
 * @brief Kalman Filter for tracking bounding boxes in image space (LibTorch version)
 * 
 * Works directly with torch tensors in [x1, y1, x2, y2] format.
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
     * @param bbox Initial bounding box tensor [x1, y1, x2, y2], shape [4]
     */
    explicit KalmanBoxTracker(const torch::Tensor& bbox);
    
    /**
     * @brief Update the state with a new measurement
     * @param bbox Bounding box tensor [x1, y1, x2, y2], shape [4]
     */
    void update(const torch::Tensor& bbox);
    
    /**
     * @brief Predict the next state
     * @return Predicted bounding box tensor [x1, y1, x2, y2], shape [4]
     */
    torch::Tensor predict();
    
    /**
     * @brief Get the current state as bounding box
     * @return Current bounding box tensor [x1, y1, x2, y2], shape [4]
     */
    torch::Tensor getState() const;
    
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
     * @param bbox Bounding box tensor [x1, y1, x2, y2]
     * @return State vector
     */
    torch::Tensor bboxToZ(const torch::Tensor& bbox) const;
    
    /**
     * @brief Convert state vector to bounding box
     * @param x State vector [x, y, s, r, ...]
     * @return Bounding box tensor [x1, y1, x2, y2]
     */
    torch::Tensor xToBbox(const torch::Tensor& x) const;
    
    // Kalman filter matrices (LibTorch tensors)
    torch::Tensor F_; // State transition matrix (7x7)
    torch::Tensor H_; // Measurement matrix (4x7)
    torch::Tensor Q_; // Process noise covariance (7x7)
    torch::Tensor R_; // Measurement noise covariance (4x4)
    torch::Tensor P_; // State covariance (7x7)
    
    // State
    torch::Tensor x_; // State vector (7x1)
    
    // Tracking metadata
    int id_;
    int time_since_update_;
    int hit_streak_;
    int age_;
    
    static int count_; // Global counter for unique IDs
};

/**
 * @brief SORT Tracker - Simple Online and Realtime Tracking (LibTorch version)
 * 
 * Works directly with torch tensors for seamless integration with PyTorch models.
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
     * @param detections Tensor of bounding boxes [N, 4] in [x1, y1, x2, y2] format
     * @return Tracked objects: tensor [M, 5] where each row is [x1, y1, x2, y2, track_id]
     */
    torch::Tensor update(const torch::Tensor& detections);
    
    /**
     * @brief Get current active track IDs in order of detections
     * @return Tensor of track IDs [N]
     */
    torch::Tensor getTrackIds() const;
    
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
     * @brief Associate detections to existing trackers using Hungarian algorithm
     * @param detections Tensor of detection boxes [N, 4]
     * @param trackers Tensor of tracker predictions [M, 4]
     * @param matched Indices of matched pairs (detection_idx, tracker_idx)
     * @param unmatched_dets Indices of unmatched detections
     * @param unmatched_trks Indices of unmatched trackers
     */
    void associateDetectionsToTrackers(
        const torch::Tensor& detections,
        const torch::Tensor& trackers,
        std::vector<std::pair<int, int>>& matched,
        std::vector<int>& unmatched_dets,
        std::vector<int>& unmatched_trks);
    
    /**
     * @brief Solve linear assignment problem using Hungarian algorithm
     * @param cost_matrix Cost matrix (higher cost = worse match)
     * @return Vector of matched pairs (row_idx, col_idx)
     */
    std::vector<std::pair<int, int>> hungarianAlgorithm(
        const torch::Tensor& cost_matrix);
    
    // Tracker parameters
    int max_age_;
    int min_hits_;
    double iou_threshold_;
    
    // Active trackers
    std::vector<std::unique_ptr<KalmanBoxTracker>> trackers_;
    
    // Track IDs from last update
    torch::Tensor last_track_ids_;
    
    // Frame counter
    int frame_count_;
};

} // namespace libtorch
} // namespace buttoning_system

#endif // SORT_TRACKER_LIBTORCH_HPP_
