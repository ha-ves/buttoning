# SORT Tracker LibTorch Migration Summary

## Overview

Successfully converted the SORT (Simple Online and Realtime Tracking) tracker from using Eigen linear algebra library to LibTorch (PyTorch C++ API).

## Files Modified

### 1. Header File
**File**: `include/buttoning_system/sort_tracker.hpp`

**Changes**:
- Replaced `#include <Eigen/Dense>` with `#include <torch/torch.h>`
- Changed all `Eigen::VectorXd` and `Eigen::MatrixXd` to `torch::Tensor`
- Updated function signatures to use `torch::Tensor` instead of Eigen types

### 2. Implementation File
**File**: `src/sort_tracker.cpp`

**Changes**:
- Converted Kalman filter matrices from Eigen to torch tensors
- Updated all matrix operations:
  - `A * B` → `torch::matmul(A, B)`
  - `A.transpose()` → `A.transpose(0, 1)`
  - `A.inverse()` → `torch::inverse(A)`
  - Element access: `A(i,j)` → `A[i][j]` or accessor
  - Slicing: `A.block<n,m>(i,j)` → `A.slice(0, i, i+n).slice(1, j, j+m)`
- All tensors use `torch::kFloat64` for numerical stability

### 3. CMakeLists.txt
**File**: `CMakeLists.txt`

**Changes**:
- Replaced `find_package(Eigen3 REQUIRED)` with `find_package(Torch REQUIRED)`
- Updated include directories: `${EIGEN3_INCLUDE_DIRS}` → `${TORCH_INCLUDE_DIRS}`
- Added `${TORCH_LIBRARIES}` to link libraries for `detection_node`
- Removed Eigen3::Eigen from `hand_detection_node` target
- Added new test executable `test_libtorch_sort`

## New Files Created

### 1. Documentation
- **README_LIBTORCH.md**: Complete guide to the LibTorch implementation
- **LIBTORCH_SETUP.md**: Windows installation and setup guide

### 2. Test Program
- **test_libtorch_sort.cpp**: Comprehensive test suite for verifying LibTorch integration

## Key Implementation Details

### Tensor Dimensions

All Kalman filter matrices are 2D tensors:
- State vector `x_`: 7×1 tensor
- State transition `F_`: 7×7 tensor
- Measurement matrix `H_`: 4×7 tensor
- Process noise `Q_`: 7×7 tensor
- Measurement noise `R_`: 4×4 tensor
- State covariance `P_`: 7×7 tensor

### Kalman Filter Operations

**Prediction Step**:
```cpp
x_ = torch::matmul(F_, x_);
P_ = torch::matmul(torch::matmul(F_, P_), F_.transpose(0, 1)) + Q_;
```

**Update Step**:
```cpp
torch::Tensor y = z - torch::matmul(H_, x_);
torch::Tensor S = torch::matmul(torch::matmul(H_, P_), H_.transpose(0, 1)) + R_;
torch::Tensor K = torch::matmul(torch::matmul(P_, H_.transpose(0, 1)), torch::inverse(S));
x_ = x_ + torch::matmul(K, y);
P_ = torch::matmul((I - torch::matmul(K, H_)), P_);
```

### Hungarian Algorithm

Cost matrix is now a torch tensor with accessor for efficient element access:
```cpp
torch::Tensor iou_matrix = torch::zeros({detections.size(), trackers.size()}, torch::kFloat64);
auto iou_accessor = iou_matrix.accessor<double, 2>();
iou_accessor[i][j] = cost_value;
```

## API Compatibility

The public API remains **100% unchanged**:
- Same constructor signatures
- Same method names and parameters
- Same return types
- Existing code using `SortTracker` requires no modifications

Example usage:
```cpp
SortTracker tracker(3, 3, 0.3);
auto tracked = tracker.update(detections);
```

## Benefits of LibTorch

1. **Unified Framework**: Use PyTorch for both detection and tracking
2. **GPU Acceleration**: Can move tensors to CUDA for large-scale processing
3. **Modern C++ API**: Better type safety and expressiveness
4. **Active Development**: LibTorch is actively maintained by PyTorch team
5. **Deep Learning Integration**: Seamless integration with neural network models

## Migration Considerations

### Performance
- CPU performance is similar to Eigen for small matrices
- GPU acceleration requires explicit device management
- For typical tracking (< 50 objects), performance difference is negligible

### Dependencies
- Eigen: ~50 KB headers only
- LibTorch: ~500 MB library with BLAS/LAPACK optimizations
- Trade-off: Larger dependency but more features

### Memory
- Both use row-major layout by default
- LibTorch has additional overhead for autograd graph (not used here)
- Can be optimized with `torch::NoGradGuard` if needed

## Testing

The test program verifies:
1. Basic tensor operations (matmul, inverse, slicing)
2. Single tracker prediction and update
3. Multi-object tracking with ID consistency
4. Track lifecycle management

Run with:
```powershell
.\install\lib\buttoning_system\test_libtorch_sort.exe
```

## Build Instructions

1. **Install LibTorch**: Follow `LIBTORCH_SETUP.md`
2. **Set CMAKE_PREFIX_PATH**: Point to LibTorch installation
3. **Build**: `colcon build --packages-select buttoning_system`
4. **Test**: Run test executable to verify

## Compatibility

- **Windows**: Tested on Windows 10/11
- **Linux**: Should work with Linux LibTorch builds (adjust paths)
- **ROS2**: Compatible with ROS2 Humble and later
- **OpenCV**: No changes to OpenCV integration

## Future Enhancements

Potential improvements:
1. **GPU Acceleration**: Move tensors to CUDA for batch processing
2. **Mixed Precision**: Use float32 instead of float64 for speed
3. **Batch Processing**: Process multiple frames in parallel
4. **JIT Compilation**: Compile Kalman filter with TorchScript
5. **Custom Ops**: Implement custom CUDA kernels for hot paths

## Rollback

To revert to Eigen implementation:
1. Restore original files from git
2. Update CMakeLists.txt to use `find_package(Eigen3)`
3. Rebuild the package

Git commands:
```bash
git checkout HEAD -- include/buttoning_system/sort_tracker.hpp
git checkout HEAD -- src/sort_tracker.cpp
git checkout HEAD -- CMakeLists.txt
```

## Conclusion

The migration to LibTorch was successful with:
- ✅ All functionality preserved
- ✅ API compatibility maintained
- ✅ Tests passing
- ✅ Documentation complete
- ✅ Build system updated

The tracker is now ready for integration with PyTorch-based detection models and has the foundation for future GPU acceleration if needed.
