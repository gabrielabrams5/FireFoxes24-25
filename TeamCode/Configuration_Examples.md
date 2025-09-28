# FTC Robot Configuration Examples for Camera System

This document provides example robot configurations for the camera system.

## Basic Camera Configuration

### Hardware Configuration (Driver Station)

1. **Go to "Configure Robot" in Driver Station**
2. **Add the following devices:**

```xml
<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<Robot type="FirstInspires-FTC">
    <!-- Control Hub -->
    <LynxModule name="Control Hub" serialNumber="DQ1234567890" parentModuleAddress="0">
        
        <!-- Motors -->
        <LynxEmbeddedIMU name="imu" port="0" bus="0" />
        
        <DcMotor name="leftFront" port="0" />
        <DcMotor name="rightFront" port="1" />
        <DcMotor name="leftBack" port="2" />
        <DcMotor name="rightBack" port="3" />
        
        <!-- Example subsystem motors -->
        <DcMotor name="liftMotor" port="4" />
        <DcMotor name="intakeMotor" port="5" />
        
        <!-- Servos -->
        <Servo name="clawServo" port="0" />
        <Servo name="twistServo" port="1" />
        
        <!-- Cameras -->
        <Webcam name="Webcam 1" port="0" serialNumber="12345678" />
        
    </LynxModule>
    
    <!-- Expansion Hub (if used) -->
    <LynxModule name="Expansion Hub" serialNumber="DQ0987654321" parentModuleAddress="1">
        
        <!-- Additional motors/sensors -->
        <DcMotor name="extensionMotor" port="0" />
        
        <!-- Backup camera (optional) -->
        <Webcam name="Webcam 2" port="0" serialNumber="87654321" />
        
    </LynxModule>
    
</Robot>
```

## Camera Hardware Recommendations

### Primary Camera Options

#### Logitech C920 (Recommended)
- **Resolution**: 1920x1080, 1280x720, 640x480
- **Frame Rate**: Up to 30 FPS
- **Features**: Auto-focus, good low-light performance
- **Configuration Name**: "Webcam 1"

#### Logitech C922 (Premium)
- **Resolution**: 1920x1080, 1280x720, 640x480
- **Frame Rate**: Up to 60 FPS at 720p
- **Features**: Auto-focus, background removal, excellent image quality
- **Configuration Name**: "Webcam 1"

#### Logitech C270 (Budget)
- **Resolution**: 1280x720, 640x480
- **Frame Rate**: Up to 30 FPS
- **Features**: Fixed focus, basic image quality
- **Configuration Name**: "Webcam 1"

### Camera Mounting Specifications

```java
// Physical mounting constants (adjust for your robot)
public static final double CAMERA_HEIGHT = 8.0;         // inches from ground
public static final double CAMERA_FORWARD_OFFSET = 6.0;  // inches from robot center
public static final double CAMERA_TILT_ANGLE = 20.0;     // degrees down from horizontal
```

## Configuration Templates

### Template 1: Competition Robot
```java
// In your OpMode initialization
private void initializeHardware() {
    // Drive motors
    leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
    rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
    leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
    rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
    
    // Subsystem motors (example)
    liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
    intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    
    // Servos (example)
    clawServo = hardwareMap.get(Servo.class, "clawServo");
    
    // Camera system
    camera = new CameraSubsystem(hardwareMap, telemetry);
    
    // Configure camera for competition
    camera.setAprilTagEnabled(true);
    camera.setColorDetectionEnabled(true);
    camera.setTargetColor(ColorRange.YELLOW); // Adjust for game pieces
}
```

### Template 2: Testing/Development Robot
```java
private void initializeTestHardware() {
    // Minimal drive system
    leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
    rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
    leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
    rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
    
    // Camera system with debug features
    camera = new CameraSubsystem(hardwareMap, telemetry);
    camera.setAprilTagEnabled(true);
    camera.setColorDetectionEnabled(true);
    
    // Enable all processing for testing
    camera.setPredominantColorEnabled(true);
}
```

## AprilTag Setup

### Printing AprilTags
1. Download official FTC AprilTag images
2. Print on white paper with black ink
3. Ensure tags are exactly 6.5" x 6.5"
4. Mount on rigid backing (foam board, cardboard)
5. Keep tags flat and undamaged

### AprilTag Placement for Testing
```java
// Example field positions (adjust for your field setup)
public static final Pose2d APRILTAG_1_POSE = new Pose2d(60, 42, Math.toRadians(180));
public static final Pose2d APRILTAG_2_POSE = new Pose2d(60, 0, Math.toRadians(180));
public static final Pose2d APRILTAG_3_POSE = new Pose2d(60, -42, Math.toRadians(180));
```

## Color Calibration Setup

### Game Piece Colors (Example)
```java
// Adjust these based on actual game pieces and lighting
public static final ColorRange SAMPLE_YELLOW = ColorRange.YELLOW;
public static final ColorRange SAMPLE_RED = ColorRange.RED;
public static final ColorRange SAMPLE_BLUE = ColorRange.BLUE;

// Custom color ranges (if needed)
public static final ColorRange CUSTOM_YELLOW = new ColorRange(
    ColorSpace.HSV,
    new Scalar(20, 100, 100),  // Lower bound
    new Scalar(30, 255, 255)   // Upper bound
);
```

### Lighting Conditions
```java
// Camera settings for different lighting conditions
public class CameraSettings {
    // Bright field conditions
    public static final int BRIGHT_EXPOSURE = 6;
    public static final int BRIGHT_GAIN = 250;
    
    // Darker field conditions
    public static final int DARK_EXPOSURE = 20;
    public static final int DARK_GAIN = 250;
    
    // Variable lighting (use auto exposure)
    public static final boolean AUTO_EXPOSURE = true;
}
```

## Testing Checklist

### Hardware Setup
- [ ] Camera securely mounted
- [ ] USB cable properly connected
- [ ] Camera appears in robot configuration
- [ ] No loose connections
- [ ] Camera lens clean and unobstructed

### Software Configuration
- [ ] CameraSubsystem instantiated correctly
- [ ] Hardware map names match configuration
- [ ] Vision processors enabled as needed
- [ ] Telemetry displays camera status
- [ ] FTC Dashboard shows camera stream

### Functional Testing
- [ ] Camera stream visible in Dashboard
- [ ] AprilTag detection working at various distances
- [ ] Color detection working with game pieces
- [ ] Vision-guided movements smooth and accurate
- [ ] Auto-align features functioning
- [ ] Manual override always available

### Competition Preparation
- [ ] Tested under competition lighting
- [ ] Backup manual controls verified
- [ ] Performance acceptable under load
- [ ] Exposure settings optimized
- [ ] Error handling robust

## Troubleshooting Common Issues

### "Camera not found" Error
```java
// Add error handling in camera initialization
try {
    camera = new CameraSubsystem(hardwareMap, telemetry);
} catch (Exception e) {
    telemetry.addLine("Camera initialization failed: " + e.getMessage());
    telemetry.addLine("Check hardware configuration and connections");
    // Continue without camera or use fallback
}
```

### Poor Detection Performance
```java
// Optimize camera settings for your conditions
public static void optimizeForField() {
    // Reduce resolution for better performance
    CAMERA_WIDTH = 320;
    CAMERA_HEIGHT = 240;
    
    // Lower FPS if needed
    MAX_FPS = 15;
    
    // Use manual exposure for consistency
    AUTO_EXPOSURE = false;
    EXPOSURE_MS = 8;
    GAIN = 200;
}
```

### Memory/Performance Issues
```java
// Disable unused processors
camera.setAprilTagEnabled(true);     // Keep only what you need
camera.setColorDetectionEnabled(false);
camera.setPredominantColorEnabled(false);

// Optimize telemetry updates
if (loopCounter % 10 == 0) {  // Update every 10 loops
    camera.updateTelemetry();
}
```

## Integration Examples

### With Existing Autonomous
```java
// Add vision to existing autonomous
@Autonomous(name = "My Autonomous + Vision")
public class MyAutonomousWithVision extends LinearOpMode {
    
    private CameraSubsystem camera;
    private MecanumDrive drive;
    
    @Override
    public void runOpMode() {
        // Initialize existing systems
        drive = new MecanumDrive(hardwareMap, startPose);
        
        // Add camera system
        camera = new CameraSubsystem(hardwareMap, telemetry);
        camera.setAprilTagEnabled(true);
        
        waitForStart();
        
        // Existing autonomous with vision enhancements
        Actions.runBlocking(
            new SequentialAction(
                // Existing movements
                drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(30, 30), 0)
                    .build(),
                
                // Add vision-guided precision
                new VisionAlignmentAction(1, 12.0, 2.0, 1.0),
                
                // Continue with existing sequence
                // ...
            )
        );
    }
}
```

### With Existing TeleOp
```java
// Enhance existing TeleOp with vision assistance
@TeleOp(name = "My TeleOp + Vision")
public class MyTeleOpWithVision extends LinearOpMode {
    
    private CameraSubsystem camera;
    // ... existing subsystems
    
    @Override
    public void runOpMode() {
        // Initialize existing systems
        initializeRobot();
        
        // Add camera
        camera = new CameraSubsystem(hardwareMap, telemetry);
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Existing TeleOp logic
            handleDriving();
            handleSubsystems();
            
            // Add vision assistance
            if (gamepad1.left_trigger > 0.5) {
                performVisionAssist();
            }
            
            telemetry.update();
        }
    }
    
    private void performVisionAssist() {
        camera.updateAprilTagDetections();
        AprilTagDetection target = camera.getClosestAprilTag();
        
        if (target != null) {
            // Add assistance logic
            double bearing = camera.getHeadingToTarget(target);
            // Blend with manual control
        }
    }
}
```
