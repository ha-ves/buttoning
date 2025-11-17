# LibTorch Setup Guide for Windows

## Download LibTorch and TorchVision

The LibTorch SORT tracker uses **TorchVision's optimized `box_iou`** function for better performance and GPU support.

### 1. Download LibTorch

1. Visit: https://pytorch.org/get-started/locally/

2. Select the following options:
   - PyTorch Build: **Stable**
   - Your OS: **Windows**
   - Package: **LibTorch**
   - Language: **C++/Java**
   - Compute Platform: **CPU** or **CUDA 11.8/12.1** (depending on your GPU)

3. Download the Release version (not Debug) for better performance

4. Example download links:
   - **CPU only**: `libtorch-win-shared-with-deps-2.1.0+cpu.zip`
   - **CUDA 12.1**: `libtorch-win-shared-with-deps-2.1.0+cu121.zip`

## Installation Steps

### Option 1: Extract to User Directory (Recommended)

```powershell
# Create a directory for LibTorch
New-Item -ItemType Directory -Force -Path C:\libtorch

# Extract the downloaded zip file
Expand-Archive -Path .\libtorch-win-shared-with-deps-*.zip -DestinationPath C:\

# Verify installation
Test-Path C:\libtorch\share\cmake\Torch\TorchConfig.cmake
```

### Option 2: Extract to Project Directory

```powershell
# Extract to your workspace
```powershell
Expand-Archive -Path .\libtorch-win-shared-with-deps-*.zip -DestinationPath C:\path\to\install\
```
```

### 2. Download TorchVision

TorchVision C++ library is required for optimized IoU operations.

**Pre-built binaries** (recommended):
- Visit: https://github.com/pytorch/vision/releases
- Download the appropriate version matching your LibTorch version
- Example: `torchvision-0.16.0-win.zip` for LibTorch 2.1.0

**Or build from source**:
```powershell
# Clone TorchVision
git clone https://github.com/pytorch/vision.git
cd vision

# Build and install
mkdir build; cd build
cmake .. -DCMAKE_PREFIX_PATH=C:\libtorch -DCMAKE_INSTALL_PREFIX=C:\torchvision
cmake --build . --config Release
cmake --install .
```

Extract or install to `C:\torchvision`

## Configure CMake

### Method 1: Environment Variable (Recommended)

Set the CMAKE_PREFIX_PATH to include both LibTorch and TorchVision:

```powershell
# Temporary (current session only)
$env:CMAKE_PREFIX_PATH = "C:\libtorch;C:\torchvision"

# Permanent (all sessions)
[System.Environment]::SetEnvironmentVariable('CMAKE_PREFIX_PATH', 'C:\libtorch;C:\torchvision', 'User')

# Verify
echo $env:CMAKE_PREFIX_PATH
```

### Method 2: CMakeLists.txt

Add to the top of your `CMakeLists.txt`:

```cmake
# Add before find_package(Torch REQUIRED)
set(CMAKE_PREFIX_PATH "C:/libtorch;C:/torchvision")
```

### Method 3: Command Line

```powershell
colcon build --packages-select buttoning_system --cmake-args -DCMAKE_PREFIX_PATH=C:/libtorch
```

## Build the Project

```powershell
# Navigate to workspace
cd C:\path\to\your\ros2_ws

# Clean previous build (if needed)
Remove-Item -Recurse -Force build\buttoning_system, install\buttoning_system

# Build with LibTorch
colcon build --packages-select buttoning_system

# Source the workspace
.\install\setup.ps1
```

## Verify Installation

Run the test executable:

```powershell
# Run the LibTorch SORT tracker test
.\install\lib\buttoning_system\test_libtorch_sort.exe
```

Expected output:
```
╔════════════════════════════════════════════════════════════╗
║  LibTorch SORT Tracker Verification Test                  ║
╚════════════════════════════════════════════════════════════╝

[TEST] LibTorch Tensor Operations
------------------------------------------------------------
...
✓ Tensor operations test PASSED

[TEST] KalmanBoxTracker with LibTorch
------------------------------------------------------------
...
✓ KalmanBoxTracker test PASSED

[TEST] SortTracker Multi-Object Tracking
------------------------------------------------------------
...
✓ SortTracker test PASSED

------------------------------------------------------------
Test Summary: 3/3 tests passed
------------------------------------------------------------

✓ All tests PASSED! LibTorch integration successful.
```

## Troubleshooting

### Error: "Could not find a package configuration file provided by Torch"

**Cause**: CMake cannot find LibTorch

**Solution**:
```powershell
# Verify LibTorch path
Test-Path C:\libtorch\share\cmake\Torch\TorchConfig.cmake

# Set CMAKE_PREFIX_PATH
$env:CMAKE_PREFIX_PATH = "C:\libtorch"

# Rebuild
colcon build --packages-select buttoning_system
```

### Error: "DLL not found" when running executable

**Cause**: LibTorch DLLs are not in PATH

**Solution**:
```powershell
# Add LibTorch bin directory to PATH
$env:PATH += ";C:\libtorch\lib"

# Or copy DLLs to executable directory
Copy-Item C:\libtorch\lib\*.dll .\install\lib\buttoning_system\
```

### Error: "CUDA not available" but you want CPU-only

**Cause**: Downloaded CUDA version but GPU not available

**Solution**: Download CPU-only version of LibTorch and rebuild

### Build is very slow

**Cause**: Debug build or Release with debug symbols

**Solution**: Use Release build
```powershell
colcon build --packages-select buttoning_system --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## System Requirements

- **Windows 10/11** (64-bit)
- **Visual Studio 2019 or later** (for C++ compiler)
- **CMake 3.8+**
- **5 GB disk space** (for LibTorch installation)

### For CUDA version:
- **NVIDIA GPU** with compute capability 3.5+
- **CUDA Toolkit** 11.8 or 12.1
- **cuDNN** (usually included with LibTorch)

## Next Steps

After successful installation:

1. **Read the documentation**: See `README_LIBTORCH.md` for API details
2. **Run the example**: Check `sort_tracker_example.cpp`
3. **Integrate with your node**: The `detection_node` already uses the tracker
4. **Optional GPU acceleration**: See README for how to enable CUDA

## Additional Resources

- **LibTorch C++ Documentation**: https://pytorch.org/cppdocs/
- **PyTorch Tutorials**: https://pytorch.org/tutorials/advanced/cpp_frontend.html
- **SORT Paper**: https://arxiv.org/abs/1602.00763
