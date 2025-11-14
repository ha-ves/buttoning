# Build Instructions for Windows

## Prerequisites

Since this is a Windows environment, here are Windows-specific build instructions.

### Install ROS2 on Windows

1. Download ROS2 for Windows (Humble or Iron recommended):
   https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html

2. Follow the installation guide to set up Visual Studio and dependencies

### Install OpenCV

Option 1: Using vcpkg (recommended)
```powershell
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg integrate install
.\vcpkg install opencv:x64-windows
```

Option 2: Download pre-built binaries from opencv.org

### Install Additional Dependencies

```powershell
# Install cv_bridge (comes with ROS2 desktop)
# Should already be available if you installed ROS2 desktop

# For RealSense (optional)
# Download Intel RealSense SDK 2.0 for Windows
# https://github.com/IntelRealSense/librealsense/releases

# For ONNX Runtime (optional)
# Download from https://github.com/microsoft/onnxruntime/releases
```

## Building the Workspace

1. Open a new "Developer Command Prompt for VS 2019" (or 2022)

2. Set up ROS2 environment:
```powershell
cd C:\dev\ros2_humble
call install\local_setup.bat
```

3. Navigate to workspace and build:
```powershell
cd C:\Users\admin\Workspace\AIML\ros2_ws
colcon build --merge-install --packages-select buttoning_msgs buttoning_system
```

4. Source the workspace:
```powershell
call install\local_setup.bat
```

## Running on Windows

Launch the system:
```powershell
ros2 launch buttoning_system buttoning_system.launch.py
```

Or run individual nodes:
```powershell
# Terminal 1
ros2 run buttoning_system hand_detection_node

# Terminal 2  
ros2 run buttoning_system detection_node

# Terminal 3
ros2 run buttoning_system arm_controller_node
```

## Troubleshooting

### Visual Studio Build Errors
- Ensure you're using "Developer Command Prompt" not regular PowerShell
- Check that Visual Studio 2019/2022 is installed with C++ tools

### OpenCV Not Found
- Set OpenCV_DIR environment variable to your OpenCV installation
- Add OpenCV bin directory to PATH

### RealSense Issues
- Install Intel RealSense SDK 2.0
- Add RealSense bin directory to PATH
- Uncomment RealSense code in CMakeLists.txt and source files

### Python Dependencies for Launch Files
- Ensure Python 3.8+ is installed
- Launch files use Python, which comes with ROS2 Windows installation

## Notes for Windows Development

- Use backslashes or forward slashes in paths (both work in Python)
- Be careful with line endings (Git may convert LF to CRLF)
- Some Linux libraries may not be available; use Windows equivalents
- Consider WSL2 for better Linux compatibility if needed
