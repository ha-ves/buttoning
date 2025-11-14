// SORT (Simple Online and Realtime Tracking) Tracker Implementation
// Based on: "Simple Online and Realtime Tracking" by Bewley et al., 2016

#include "buttoning_system/sort_tracker.hpp"
#include <algorithm>
#include <limits>
#include <set>

namespace buttoning_system {

// Initialize static counter
int KalmanBoxTracker::count_ = 0;

//==============================================================================
// KalmanBoxTracker Implementation
//==============================================================================

KalmanBoxTracker::KalmanBoxTracker(const cv::Rect& bbox)
    : time_since_update_(0), hit_streak_(0), age_(0)
{
    // Assign unique ID
    id_ = count_++;
    
    // Initialize state vector: [x, y, s, r, vx, vy, vs]
    x_ = Eigen::VectorXd::Zero(7);
    Eigen::Vector4d z = bboxToZ(bbox);
    x_.head<4>() = z;
    
    // State transition matrix (constant velocity model)
    F_ = Eigen::MatrixXd::Identity(7, 7);
    F_(0, 4) = 1.0; // x += vx
    F_(1, 5) = 1.0; // y += vy
    F_(2, 6) = 1.0; // s += vs
    
    // Measurement matrix (we only observe position and size)
    H_ = Eigen::MatrixXd::Zero(4, 7);
    H_.block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
    
    // Measurement noise covariance
    R_ = Eigen::MatrixXd::Identity(4, 4);
    R_(2, 2) *= 10.0; // Higher uncertainty in scale
    R_(3, 3) *= 10.0; // Higher uncertainty in aspect ratio
    
    // Process noise covariance
    Q_ = Eigen::MatrixXd::Identity(7, 7);
    Q_.block<4, 4>(0, 0) *= 1.0;      // Position and size
    Q_.block<3, 3>(4, 4) *= 0.01;     // Velocities (lower noise)
    
    // Initial state covariance
    P_ = Eigen::MatrixXd::Identity(7, 7);
    P_.block<4, 4>(0, 0) *= 10.0;     // High initial uncertainty in position/size
    P_.block<3, 3>(4, 4) *= 1000.0;   // Very high uncertainty in velocities
}

Eigen::Vector4d KalmanBoxTracker::bboxToZ(const cv::Rect& bbox) const {
    double w = static_cast<double>(bbox.width);
    double h = static_cast<double>(bbox.height);
    double x = bbox.x + w / 2.0;
    double y = bbox.y + h / 2.0;
    double s = w * h; // Scale (area)
    double r = w / h; // Aspect ratio
    
    Eigen::Vector4d z;
    z << x, y, s, r;
    return z;
}

cv::Rect KalmanBoxTracker::xToBbox(const Eigen::VectorXd& x) const {
    double w = std::sqrt(x(2) * x(3));
    double h = x(2) / w;
    double x1 = x(0) - w / 2.0;
    double y1 = x(1) - h / 2.0;
    
    return cv::Rect(
        static_cast<int>(std::round(x1)),
        static_cast<int>(std::round(y1)),
        static_cast<int>(std::round(w)),
        static_cast<int>(std::round(h))
    );
}

cv::Rect KalmanBoxTracker::predict() {
    // Predict next state
    if (time_since_update_ > 0) {
        hit_streak_ = 0;
    }
    time_since_update_ += 1;
    age_ += 1;
    
    // x = F * x
    x_ = F_ * x_;
    
    // P = F * P * F' + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
    
    return getState();
}

void KalmanBoxTracker::update(const cv::Rect& bbox) {
    time_since_update_ = 0;
    hit_streak_ += 1;
    
    // Convert bbox to measurement
    Eigen::Vector4d z = bboxToZ(bbox);
    
    // Innovation (measurement residual)
    Eigen::Vector4d y = z - H_ * x_;
    
    // Innovation covariance
    Eigen::Matrix4d S = H_ * P_ * H_.transpose() + R_;
    
    // Kalman gain
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    // Update state estimate
    x_ = x_ + K * y;
    
    // Update covariance estimate
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    P_ = (I - K * H_) * P_;
}

cv::Rect KalmanBoxTracker::getState() const {
    return xToBbox(x_);
}

//==============================================================================
// SortTracker Implementation
//==============================================================================

SortTracker::SortTracker(int max_age, int min_hits, double iou_threshold)
    : max_age_(max_age), min_hits_(min_hits), iou_threshold_(iou_threshold),
      frame_count_(0)
{
}

double SortTracker::iou(const cv::Rect& bb_test, const cv::Rect& bb_gt) const {
    // Calculate intersection
    int xx1 = std::max(bb_test.x, bb_gt.x);
    int yy1 = std::max(bb_test.y, bb_gt.y);
    int xx2 = std::min(bb_test.x + bb_test.width, bb_gt.x + bb_gt.width);
    int yy2 = std::min(bb_test.y + bb_test.height, bb_gt.y + bb_gt.height);
    
    int w = std::max(0, xx2 - xx1);
    int h = std::max(0, yy2 - yy1);
    
    double intersection = static_cast<double>(w * h);
    
    // Calculate union
    double area_test = static_cast<double>(bb_test.width * bb_test.height);
    double area_gt = static_cast<double>(bb_gt.width * bb_gt.height);
    double union_area = area_test + area_gt - intersection;
    
    if (union_area <= 0.0) {
        return 0.0;
    }
    
    return intersection / union_area;
}

std::vector<std::pair<int, int>> SortTracker::hungarianAlgorithm(
    const Eigen::MatrixXd& cost_matrix)
{
    std::vector<std::pair<int, int>> matches;
    
    if (cost_matrix.rows() == 0 || cost_matrix.cols() == 0) {
        return matches;
    }
    
    // Simple greedy assignment (for small problems)
    // For production, consider using a proper Hungarian algorithm library
    int n_rows = cost_matrix.rows();
    int n_cols = cost_matrix.cols();
    
    std::set<int> matched_rows;
    std::set<int> matched_cols;
    
    // Create a list of all (cost, row, col) tuples and sort by cost
    struct Assignment {
        double cost;
        int row;
        int col;
    };
    
    std::vector<Assignment> assignments;
    for (int i = 0; i < n_rows; ++i) {
        for (int j = 0; j < n_cols; ++j) {
            assignments.push_back({cost_matrix(i, j), i, j});
        }
    }
    
    // Sort by cost (ascending for minimization)
    std::sort(assignments.begin(), assignments.end(),
              [](const Assignment& a, const Assignment& b) {
                  return a.cost < b.cost;
              });
    
    // Greedily assign matches
    for (const auto& assignment : assignments) {
        if (matched_rows.find(assignment.row) == matched_rows.end() &&
            matched_cols.find(assignment.col) == matched_cols.end()) {
            matches.push_back({assignment.row, assignment.col});
            matched_rows.insert(assignment.row);
            matched_cols.insert(assignment.col);
        }
    }
    
    return matches;
}

void SortTracker::associateDetectionsToTrackers(
    const std::vector<cv::Rect>& detections,
    const std::vector<cv::Rect>& trackers,
    std::vector<std::pair<int, int>>& matched,
    std::vector<int>& unmatched_dets,
    std::vector<int>& unmatched_trks)
{
    matched.clear();
    unmatched_dets.clear();
    unmatched_trks.clear();
    
    if (trackers.empty()) {
        for (size_t i = 0; i < detections.size(); ++i) {
            unmatched_dets.push_back(i);
        }
        return;
    }
    
    if (detections.empty()) {
        for (size_t i = 0; i < trackers.size(); ++i) {
            unmatched_trks.push_back(i);
        }
        return;
    }
    
    // Build IOU cost matrix (use 1 - IOU as cost for minimization)
    Eigen::MatrixXd iou_matrix(detections.size(), trackers.size());
    for (size_t d = 0; d < detections.size(); ++d) {
        for (size_t t = 0; t < trackers.size(); ++t) {
            iou_matrix(d, t) = 1.0 - iou(detections[d], trackers[t]);
        }
    }
    
    // Solve assignment problem
    std::vector<std::pair<int, int>> raw_matches = hungarianAlgorithm(iou_matrix);
    
    // Filter matches based on IOU threshold
    std::set<int> matched_det_indices;
    std::set<int> matched_trk_indices;
    
    for (const auto& match : raw_matches) {
        int det_idx = match.first;
        int trk_idx = match.second;
        double iou_val = 1.0 - iou_matrix(det_idx, trk_idx);
        
        if (iou_val >= iou_threshold_) {
            matched.push_back({det_idx, trk_idx});
            matched_det_indices.insert(det_idx);
            matched_trk_indices.insert(trk_idx);
        }
    }
    
    // Find unmatched detections
    for (size_t d = 0; d < detections.size(); ++d) {
        if (matched_det_indices.find(d) == matched_det_indices.end()) {
            unmatched_dets.push_back(d);
        }
    }
    
    // Find unmatched trackers
    for (size_t t = 0; t < trackers.size(); ++t) {
        if (matched_trk_indices.find(t) == matched_trk_indices.end()) {
            unmatched_trks.push_back(t);
        }
    }
}

std::vector<std::pair<int, cv::Rect>> SortTracker::update(
    const std::vector<cv::Rect>& detections,
    const std::vector<float>& scores)
{
    frame_count_ += 1;
    
    // Get predicted locations from existing trackers
    std::vector<cv::Rect> trk_predictions;
    for (auto& trk : trackers_) {
        cv::Rect pred = trk->predict();
        trk_predictions.push_back(pred);
    }
    
    // Associate detections to trackers
    std::vector<std::pair<int, int>> matched;
    std::vector<int> unmatched_dets;
    std::vector<int> unmatched_trks;
    
    associateDetectionsToTrackers(detections, trk_predictions,
                                   matched, unmatched_dets, unmatched_trks);
    
    // Update matched trackers with assigned detections
    for (const auto& match : matched) {
        int det_idx = match.first;
        int trk_idx = match.second;
        trackers_[trk_idx]->update(detections[det_idx]);
    }
    
    // Create new trackers for unmatched detections
    for (int det_idx : unmatched_dets) {
        auto new_tracker = std::make_unique<KalmanBoxTracker>(detections[det_idx]);
        trackers_.push_back(std::move(new_tracker));
    }
    
    // Remove dead trackers
    std::vector<std::unique_ptr<KalmanBoxTracker>> active_trackers;
    for (auto& trk : trackers_) {
        // Keep tracker if it's young or was recently updated
        if (trk->getTimeSinceUpdate() < max_age_) {
            active_trackers.push_back(std::move(trk));
        }
    }
    trackers_ = std::move(active_trackers);
    
    // Build return list of tracked objects
    std::vector<std::pair<int, cv::Rect>> ret;
    last_track_ids_.clear();
    
    for (auto& trk : trackers_) {
        // Only return tracks that have been confirmed (min_hits)
        if (trk->getTimeSinceUpdate() == 0 && trk->getHitStreak() >= min_hits_) {
            ret.push_back({trk->getId(), trk->getState()});
            last_track_ids_.push_back(trk->getId());
        } else if (trk->getTimeSinceUpdate() == 0) {
            // Track exists but not confirmed yet
            last_track_ids_.push_back(trk->getId());
        }
    }
    
    // For detections that got matched, store their track IDs in order
    std::vector<int> ordered_track_ids(detections.size(), -1);
    for (const auto& match : matched) {
        int det_idx = match.first;
        int trk_idx = match.second;
        ordered_track_ids[det_idx] = trackers_[trk_idx]->getId();
    }
    
    // For new detections (unmatched), assign their new track IDs
    int new_tracker_offset = trackers_.size() - unmatched_dets.size();
    for (size_t i = 0; i < unmatched_dets.size(); ++i) {
        int det_idx = unmatched_dets[i];
        ordered_track_ids[det_idx] = trackers_[new_tracker_offset + i]->getId();
    }
    
    last_track_ids_ = ordered_track_ids;
    
    return ret;
}

void SortTracker::reset() {
    trackers_.clear();
    last_track_ids_.clear();
    frame_count_ = 0;
}

} // namespace buttoning_system
