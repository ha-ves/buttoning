// SORT (Simple Online and Realtime Tracking) Tracker Implementation with LibTorch
// Based on: "Simple Online and Realtime Tracking" by Bewley et al., 2016

#include "buttoning_system/sort_tracker_libtorch.hpp"
#include <torch/torch.h>
#include <torchvision/ops/ops.h>
#include <algorithm>
#include <limits>
#include <set>

namespace buttoning_system {
namespace libtorch {

// Initialize static counter
int KalmanBoxTracker::count_ = 0;

//==============================================================================
// KalmanBoxTracker Implementation (LibTorch)
//==============================================================================

KalmanBoxTracker::KalmanBoxTracker(const torch::Tensor& bbox)
    : time_since_update_(0), hit_streak_(0), age_(0)
{
    // Assign unique ID
    id_ = count_++;
    
    // Initialize state vector: [x, y, s, r, vx, vy, vs]
    x_ = torch::zeros({7, 1}, torch::kFloat64);
    torch::Tensor z = bboxToZ(bbox);
    x_.slice(0, 0, 4) = z;
    
    // State transition matrix (constant velocity model)
    F_ = torch::eye(7, torch::kFloat64);
    F_[0][4] = 1.0; // x += vx
    F_[1][5] = 1.0; // y += vy
    F_[2][6] = 1.0; // s += vs
    
    // Measurement matrix (we only observe position and size)
    H_ = torch::zeros({4, 7}, torch::kFloat64);
    H_.slice(0, 0, 4).slice(1, 0, 4) = torch::eye(4, torch::kFloat64);
    
    // Measurement noise covariance
    R_ = torch::eye(4, torch::kFloat64);
    R_[2][2] = 10.0; // Higher uncertainty in scale
    R_[3][3] = 10.0; // Higher uncertainty in aspect ratio
    
    // Process noise covariance
    Q_ = torch::eye(7, torch::kFloat64);
    Q_.slice(0, 0, 4).slice(1, 0, 4) *= 1.0;      // Position and size
    Q_.slice(0, 4, 7).slice(1, 4, 7) *= 0.01;     // Velocities (lower noise)
    
    // Initial state covariance
    P_ = torch::eye(7, torch::kFloat64);
    P_.slice(0, 0, 4).slice(1, 0, 4) *= 10.0;     // High initial uncertainty in position/size
    P_.slice(0, 4, 7).slice(1, 4, 7) *= 1000.0;   // Very high uncertainty in velocities
}

torch::Tensor KalmanBoxTracker::bboxToZ(const torch::Tensor& bbox) const {
    // bbox is [x1, y1, x2, y2] in shape [4]
    auto bbox_acc = bbox.accessor<float, 1>();
    double x1 = bbox_acc[0];
    double y1 = bbox_acc[1];
    double x2 = bbox_acc[2];
    double y2 = bbox_acc[3];
    
    double w = x2 - x1;
    double h = y2 - y1;
    double x = x1 + w / 2.0;
    double y = y1 + h / 2.0;
    double s = w * h; // Scale (area)
    double r = w / h; // Aspect ratio
    
    torch::Tensor z = torch::zeros({4, 1}, torch::kFloat64);
    z[0][0] = x;
    z[1][0] = y;
    z[2][0] = s;
    z[3][0] = r;
    return z;
}

torch::Tensor KalmanBoxTracker::xToBbox(const torch::Tensor& x) const {
    double w = std::sqrt(x[2][0].item<double>() * x[3][0].item<double>());
    double h = x[2][0].item<double>() / w;
    double x1 = x[0][0].item<double>() - w / 2.0;
    double y1 = x[1][0].item<double>() - h / 2.0;
    double x2 = x1 + w;
    double y2 = y1 + h;
    
    return torch::tensor({x1, y1, x2, y2}, torch::kFloat32);
}

torch::Tensor KalmanBoxTracker::predict() {
    // Predict next state
    if (time_since_update_ > 0) {
        hit_streak_ = 0;
    }
    time_since_update_ += 1;
    age_ += 1;
    
    // x = F * x
    x_ = torch::matmul(F_, x_);
    
    // P = F * P * F' + Q
    P_ = torch::matmul(torch::matmul(F_, P_), F_.transpose(0, 1)) + Q_;
    
    return getState();
}

void KalmanBoxTracker::update(const torch::Tensor& bbox) {
    time_since_update_ = 0;
    hit_streak_ += 1;
    
    // Convert bbox to measurement
    torch::Tensor z = bboxToZ(bbox);
    
    // Innovation (measurement residual)
    torch::Tensor y = z - torch::matmul(H_, x_);
    
    // Innovation covariance
    torch::Tensor S = torch::matmul(torch::matmul(H_, P_), H_.transpose(0, 1)) + R_;
    
    // Kalman gain
    torch::Tensor K = torch::matmul(torch::matmul(P_, H_.transpose(0, 1)), torch::inverse(S));
    
    // Update state estimate
    x_ = x_ + torch::matmul(K, y);
    
    // Update covariance estimate
    torch::Tensor I = torch::eye(7, torch::kFloat64);
    P_ = torch::matmul((I - torch::matmul(K, H_)), P_);
}

torch::Tensor KalmanBoxTracker::getState() const {
    return xToBbox(x_);
}

//==============================================================================
// SortTracker Implementation (LibTorch)
//==============================================================================

SortTracker::SortTracker(int max_age, int min_hits, double iou_threshold)
    : max_age_(max_age), min_hits_(min_hits), iou_threshold_(iou_threshold),
      frame_count_(0)
{
    last_track_ids_ = torch::empty({0}, torch::kInt64);
}

std::vector<std::pair<int, int>> SortTracker::hungarianAlgorithm(
    const torch::Tensor& cost_matrix)
{
    std::vector<std::pair<int, int>> matches;
    
    if (cost_matrix.size(0) == 0 || cost_matrix.size(1) == 0) {
        return matches;
    }
    
    // Simple greedy assignment (for small problems)
    // For production, consider using a proper Hungarian algorithm library
    int n_rows = cost_matrix.size(0);
    int n_cols = cost_matrix.size(1);
    
    std::set<int> matched_rows;
    std::set<int> matched_cols;
    
    // Create a list of all (cost, row, col) tuples and sort by cost
    struct Assignment {
        double cost;
        int row;
        int col;
    };
    
    std::vector<Assignment> assignments;
    auto cost_accessor = cost_matrix.accessor<double, 2>();
    for (int i = 0; i < n_rows; ++i) {
        for (int j = 0; j < n_cols; ++j) {
            assignments.push_back({cost_accessor[i][j], i, j});
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
    const torch::Tensor& detections,
    const torch::Tensor& trackers,
    std::vector<std::pair<int, int>>& matched,
    std::vector<int>& unmatched_dets,
    std::vector<int>& unmatched_trks)
{
    matched.clear();
    unmatched_dets.clear();
    unmatched_trks.clear();
    
    int n_dets = detections.size(0);
    int n_trks = trackers.size(0);
    
    if (n_trks == 0) {
        for (int i = 0; i < n_dets; ++i) {
            unmatched_dets.push_back(i);
        }
        return;
    }
    
    if (n_dets == 0) {
        for (int i = 0; i < n_trks; ++i) {
            unmatched_trks.push_back(i);
        }
        return;
    }
    
    // Compute IoU matrix: [N_detections x N_trackers] using TorchVision
    torch::Tensor iou_matrix = vision::ops::box_iou(detections, trackers);
    
    // Convert to cost matrix (1 - IoU) for minimization
    torch::Tensor cost_matrix = 1.0 - iou_matrix.to(torch::kFloat64);
    
    // Solve assignment problem
    std::vector<std::pair<int, int>> raw_matches = hungarianAlgorithm(cost_matrix);
    
    // Filter matches based on IOU threshold
    std::set<int> matched_det_indices;
    std::set<int> matched_trk_indices;
    
    auto cost_accessor = cost_matrix.accessor<double, 2>();
    for (const auto& match : raw_matches) {
        int det_idx = match.first;
        int trk_idx = match.second;
        double iou_val = 1.0 - cost_accessor[det_idx][trk_idx];
        
        if (iou_val >= iou_threshold_) {
            matched.push_back({det_idx, trk_idx});
            matched_det_indices.insert(det_idx);
            matched_trk_indices.insert(trk_idx);
        }
    }
    
    // Find unmatched detections
    for (int d = 0; d < n_dets; ++d) {
        if (matched_det_indices.find(d) == matched_det_indices.end()) {
            unmatched_dets.push_back(d);
        }
    }
    
    // Find unmatched trackers
    for (int t = 0; t < n_trks; ++t) {
        if (matched_trk_indices.find(t) == matched_trk_indices.end()) {
            unmatched_trks.push_back(t);
        }
    }
}

torch::Tensor SortTracker::update(const torch::Tensor& detections)
{
    frame_count_ += 1;
    
    int n_dets = detections.size(0);
    
    // Get predicted locations from existing trackers
    std::vector<torch::Tensor> trk_predictions_vec;
    for (auto& trk : trackers_) {
        trk_predictions_vec.push_back(trk->predict());
    }
    
    torch::Tensor trk_predictions;
    if (!trk_predictions_vec.empty()) {
        trk_predictions = torch::stack(trk_predictions_vec, 0);
    } else {
        trk_predictions = torch::empty({0, 4}, torch::kFloat32);
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
    
    // Build return tensor of tracked objects [M, 5] where columns are [x1, y1, x2, y2, track_id]
    std::vector<torch::Tensor> confirmed_tracks;
    std::vector<int64_t> all_track_ids;
    
    for (auto& trk : trackers_) {
        // Only return tracks that have been confirmed (min_hits)
        if (trk->getTimeSinceUpdate() == 0 && trk->getHitStreak() >= min_hits_) {
            torch::Tensor state = trk->getState();  // [4]
            torch::Tensor track_id_tensor = torch::tensor({static_cast<float>(trk->getId())}, torch::kFloat32);
            torch::Tensor track_with_id = torch::cat({state, track_id_tensor}, 0);  // [5]
            confirmed_tracks.push_back(track_with_id);
            all_track_ids.push_back(trk->getId());
        } else if (trk->getTimeSinceUpdate() == 0) {
            // Track exists but not confirmed yet
            all_track_ids.push_back(trk->getId());
        }
    }
    
    // Store track IDs for reference
    last_track_ids_ = torch::tensor(all_track_ids, torch::kInt64);
    
    // Return confirmed tracks as [M, 5] tensor
    if (confirmed_tracks.empty()) {
        return torch::empty({0, 5}, torch::kFloat32);
    }
    return torch::stack(confirmed_tracks, 0);
}

torch::Tensor SortTracker::getTrackIds() const {
    return last_track_ids_.clone();
}

void SortTracker::reset() {
    trackers_.clear();
    last_track_ids_ = torch::empty({0}, torch::kInt64);
    frame_count_ = 0;
}

} // namespace libtorch
} // namespace buttoning_system
