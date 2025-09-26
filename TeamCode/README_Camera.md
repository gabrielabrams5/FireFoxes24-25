# Camera System Documentation

## Overview

This comprehensive camera system for FTC provides:
- **AprilTag Detection**: Precise robot positioning and navigation
- **Color Detection**: Object tracking and sample collection
- **Vision-Guided Autonomous**: Automated navigation using visual landmarks
- **Vision-Assisted TeleOp**: Enhanced manual control with camera assistance
- **Real-time Streaming**: Live camera feed to FTC Dashboard

## Hardware Requirements

### Supported Cameras
- **Logitech C920/C922** (Recommended)
- **Logitech C270** (Budget option)
- **Microsoft LifeCam HD-3000**
- **Generic USB UVC cameras**
- **Built-in phone camera** (as fallback)

### Hardware Configuration Names
In your robot configuration file, add:
```
Webcam 1: USB Camera (Primary)
webcam: USB Camera (Fallback)
```

### Mounting Recommendations
- **Height**: 8-12 inches from the ground
- **Angle**: Slightly tilted down (15-30 degrees)
- **Position**: Center of robot, 6+ inches forward of rotation center
- **Protection**: Consider lens protection for game pieces

## OpMode Guide

### 1. Camera Stream OpMode
**File**: `CameraStreamOpMode.java`
**Purpose**: Basic camera streaming and configuration

**Controls**:
- START/BACK: Start/stop streaming
- A/B: Toggle AprilTag/Color detection overlays
- X/Y/LB/RB: Change target colors
- D-pad: Adjust exposure settings
- D-pad up/down: Change stream FPS

**Usage**:
1. Run this OpMode first to verify camera is working
2. Adjust exposure settings for your lighting conditions
3. Use FTC Dashboard to view camera feed

### 2. AprilTag Detection OpMode
**File**: `AprilTagDetectionOpMode.java`
**Purpose**: Test and calibrate AprilTag detection

**Controls**:
- Left stick: Manual drive
- Right stick: Manual turn
- D-pad up/down: Change target tag ID
- D-pad left: Toggle show all tags
- D-pad right: Toggle auto-align
- A/B: Exposure controls

**Tunable Parameters**:
```java
public static int TARGET_TAG_ID = 1;
public static double ALIGNMENT_TOLERANCE = 2.0; // degrees
public static double DISTANCE_TOLERANCE = 2.0;  // inches
```

### 3. Color Detection OpMode
**File**: `ColorDetectionOpMode.java`
**Purpose**: Test and calibrate color detection

**Controls**:
- Left stick: Manual drive
- Right stick: Manual turn
- A/B/X/Y: Change target colors
- Left/Right bumper: Toggle tracking modes
- D-pad: Exposure controls

**Tunable Parameters**:
```java
public static ColorRange TARGET_COLOR = ColorRange.BLUE;
public static double MIN_BLOB_AREA = 100.0;
public static double ALIGNMENT_TOLERANCE = 5.0; // degrees
```

### 4. Vision-Guided Autonomous
**File**: `VisionGuidedAutonomous.java`
**Purpose**: Fully autonomous vision-based navigation

**Features**:
- AprilTag approach and alignment
- Color sample detection and collection
- Vision-guided path planning
- Automatic error recovery

**Tunable Parameters**:
```java
public static double APPROACH_DISTANCE = 12.0;    // inches
public static double ALIGNMENT_TOLERANCE = 2.0;   // degrees
public static int TARGET_APRILTAG_ID = 1;
public static ColorRange SAMPLE_COLOR = ColorRange.YELLOW;
```

### 5. Vision-Assisted TeleOp
**File**: `VisionAssistedTeleOp.java`
**Purpose**: Enhanced manual control with vision assistance

**Controls**:
- **Normal driving**: Left stick (drive/strafe), Right stick (turn)
- **Left trigger**: AprilTag alignment assist
- **Right trigger**: AprilTag approach assist
- **Left bumper**: Color tracking assist
- **Right bumper**: Color approach assist
- **D-pad down**: Precision mode
- **A/B**: Toggle assist features
- **X/Y**: Change target colors

## Configuration Parameters

### Camera Settings
```java
// In CameraSubsystem.java
public static int CAMERA_WIDTH = 640;
public static int CAMERA_HEIGHT = 480;
public static int MAX_FPS = 30;
public static boolean AUTO_EXPOSURE = true;
public static int EXPOSURE_MS = 6;        // For bright conditions
public static int GAIN = 250;
```

### AprilTag Settings
```java
public static double APRIL_TAG_SIZE = 0.166;  // 6.5 inches in meters
public static int DESIRED_TAG_ID = -1;        // -1 for any tag
```

### Color Detection Settings
```java
public static double COLOR_BLOB_MIN_AREA = 50.0;
public static double COLOR_BLOB_MAX_AREA = 10000.0;
public static double COLOR_EROSION = 10.0;
public static double COLOR_DILATION = 10.0;
```

## Calibration Guide

### 1. Camera Calibration
1. Run `CameraStreamOpMode`
2. Adjust exposure for consistent lighting:
   - **Bright field**: Exposure 6ms, Gain 250
   - **Darker field**: Exposure 20ms, Gain 250
   - **Auto exposure**: Good for varying conditions
3. Verify image quality in FTC Dashboard

### 2. AprilTag Calibration
1. Print AprilTags from [FTC Documentation](https://ftc-docs.firstinspires.org/apriltag-intro)
2. Place tags at known distances (12", 24", 36")
3. Run `AprilTagDetectionOpMode`
4. Verify distance accuracy
5. Adjust lens intrinsics if needed:
```java
.setLensIntrinsics(fx, fy, cx, cy)
```

### 3. Color Calibration
1. Run `ColorDetectionOpMode`
2. Test with game pieces under field lighting
3. Adjust color ranges if needed:
   - Try different ColorRange options
   - Adjust MIN_BLOB_AREA to filter noise
   - Tune erosion/dilation for cleaner detection

### 4. Vision Navigation Calibration
1. Measure and set camera offset from robot center:
```java
public static double CAMERA_FORWARD_OFFSET = 6.0; // inches
public static double CAMERA_HEIGHT = 8.0;         // inches
```
2. Test approach distances and adjust tolerances
3. Tune PID controllers for smooth movement

## Troubleshooting

### Camera Not Detected
- Check USB connections
- Verify hardware configuration names
- Try different USB ports
- Check camera compatibility

### Poor AprilTag Detection
- Improve lighting conditions
- Reduce camera exposure
- Ensure AprilTags are flat and undamaged
- Check for motion blur (reduce exposure)

### Inconsistent Color Detection
- Calibrate under field lighting
- Adjust camera exposure
- Use manual exposure for consistent conditions
- Filter blobs by size and shape

### Performance Issues
- Reduce camera resolution
- Lower FPS
- Disable unused processors
- Optimize telemetry updates

### Vision Guidance Not Working
- Verify coordinate system orientation
- Check PID controller gains
- Ensure proper robot pose initialization
- Test individual components separately

## Advanced Features

### Custom Vision Processors
Create custom processors by extending OpenCV VisionProcessor:
```java
public class CustomProcessor extends VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize processor
    }
    
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Process frame and return results
        return results;
    }
}
```

### Multi-Camera Setup
For robots with multiple cameras:
```java
// Create separate CameraSubsystem instances
CameraSubsystem frontCamera = new CameraSubsystem(hardwareMap, "front_camera");
CameraSubsystem backCamera = new CameraSubsystem(hardwareMap, "back_camera");
```

### Field Coordinate Integration
Use AprilTag detections for field positioning:
```java
// Known AprilTag positions on field
Pose2d knownTagPose = new Pose2d(72, 36, 0); // Example coordinates
Pose2d robotPose = VisionNavigation.calculateRobotPoseFromTag(detection, knownTagPose);
```

## Competition Tips

1. **Test Early**: Verify vision system works in competition lighting
2. **Have Backups**: Include manual fallbacks for all vision features
3. **Monitor Performance**: Watch for processing delays and frame drops
4. **Tune Conservatively**: Prefer reliability over perfect precision
5. **Practice Transitions**: Smooth handoffs between vision and manual control

## Support Resources

- [FTC Vision Documentation](https://ftc-docs.firstinspires.org/apriltag-intro)
- [OpenCV Documentation](https://docs.opencv.org/)
- [FTC Dashboard](https://acmerobotics.github.io/ftc-dashboard/)
- Team forums and Discord channels

## File Structure
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── subsystems/
│   └── CameraSubsystem.java
├── vision/
│   ├── AprilTagDetectionOpMode.java
│   ├── ColorDetectionOpMode.java
│   ├── CameraStreamOpMode.java
│   ├── VisionGuidedAutonomous.java
│   ├── VisionAssistedTeleOp.java
│   └── utils/
│       ├── VisionNavigation.java
│       └── VisionMath.java
└── README_Camera.md (this file)
```
