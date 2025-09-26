package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

@Config
@TeleOp(name = "Camera Stream", group = "Vision")
public class CameraStreamOpMode extends LinearOpMode {
    
    // Tunable parameters
    public static int STREAM_FPS = 30;
    public static boolean SHOW_APRILTAGS = false;
    public static boolean SHOW_COLOR_DETECTION = false;
    public static boolean AUTO_EXPOSURE = true;
    public static int MANUAL_EXPOSURE_MS = 6;
    public static int MANUAL_GAIN = 250;
    
    private CameraSubsystem camera;
    private ElapsedTime runtime = new ElapsedTime();
    private FtcDashboard dashboard;
    
    @Override
    public void runOpMode() {
        // Initialize telemetry for dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        
        // Initialize camera
        camera = new CameraSubsystem(hardwareMap, telemetry);
        camera.setAprilTagEnabled(SHOW_APRILTAGS);
        camera.setColorDetectionEnabled(SHOW_COLOR_DETECTION);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Camera Stream OpMode");
        telemetry.addLine("• View camera feed on FTC Dashboard");
        telemetry.addLine("• Use gamepad to control camera settings");
        telemetry.addLine("• Toggle overlays and processing features");
        telemetry.addLine();
        telemetry.addLine("Dashboard URL:");
        telemetry.addLine("http://192.168.43.1:8080/dash (Control Hub)");
        telemetry.addLine("http://192.168.49.1:8080/dash (Driver Station)");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            // Update camera detections if enabled
            if (SHOW_APRILTAGS) {
                camera.updateAprilTagDetections();
            }
            if (SHOW_COLOR_DETECTION) {
                camera.updateColorDetections();
            }
            
            // Display runtime information
            telemetry.addData("Status", "Streaming");
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.addData("Stream FPS", STREAM_FPS);
            telemetry.addLine();
            
            // Camera settings
            telemetry.addData("Camera Settings", "");
            telemetry.addData("  Auto Exposure", AUTO_EXPOSURE ? "ON" : "OFF");
            if (!AUTO_EXPOSURE) {
                telemetry.addData("  Manual Exposure", "%d ms", MANUAL_EXPOSURE_MS);
                telemetry.addData("  Manual Gain", MANUAL_GAIN);
            }
            telemetry.addLine();
            
            // Processing settings
            telemetry.addData("Processing Overlays", "");
            telemetry.addData("  AprilTag Detection", SHOW_APRILTAGS ? "ON" : "OFF");
            telemetry.addData("  Color Detection", SHOW_COLOR_DETECTION ? "ON" : "OFF");
            telemetry.addLine();
            
            // Handle controls
            handleCameraControls();
            handleProcessingControls();
            handleExposureControls();
            
            // Update camera telemetry
            camera.updateTelemetry();
            
            // Performance monitoring
            displayPerformanceInfo();
            
            telemetry.update();
            
            // Small delay to prevent overwhelming the system
            sleep(33); // ~30 FPS telemetry updates
        }
        
        // Cleanup
        camera.close();
    }
    
    private void handleCameraControls() {
        // Basic camera controls
        if (gamepad1.start) {
            camera.startStreaming();
            telemetry.addLine("✓ Camera streaming started");
        }
        
        if (gamepad1.back) {
            camera.stopStreaming();
            telemetry.addLine("✗ Camera streaming stopped");
        }
        
        // Stream quality controls
        if (gamepad1.dpad_up) {
            STREAM_FPS = Math.min(STREAM_FPS + 5, 60);
            // Note: FPS change requires VisionPortal restart in real implementation
        }
        
        if (gamepad1.dpad_down) {
            STREAM_FPS = Math.max(STREAM_FPS - 5, 5);
            // Note: FPS change requires VisionPortal restart in real implementation
        }
    }
    
    private void handleProcessingControls() {
        // Toggle AprilTag detection overlay
        if (gamepad1.a) {
            SHOW_APRILTAGS = !SHOW_APRILTAGS;
            camera.setAprilTagEnabled(SHOW_APRILTAGS);
            telemetry.addLine(String.format("AprilTag detection: %s", 
                SHOW_APRILTAGS ? "ENABLED" : "DISABLED"));
        }
        
        // Toggle color detection overlay
        if (gamepad1.b) {
            SHOW_COLOR_DETECTION = !SHOW_COLOR_DETECTION;
            camera.setColorDetectionEnabled(SHOW_COLOR_DETECTION);
            telemetry.addLine(String.format("Color detection: %s", 
                SHOW_COLOR_DETECTION ? "ENABLED" : "DISABLED"));
        }
        
        // Cycle through color targets (when color detection is enabled)
        if (SHOW_COLOR_DETECTION) {
            if (gamepad1.x) {
                camera.setTargetColor(org.firstinspires.ftc.vision.opencv.ColorRange.RED);
                telemetry.addLine("Target color: RED");
            }
            if (gamepad1.y) {
                camera.setTargetColor(org.firstinspires.ftc.vision.opencv.ColorRange.BLUE);
                telemetry.addLine("Target color: BLUE");
            }
            if (gamepad1.left_bumper) {
                camera.setTargetColor(org.firstinspires.ftc.vision.opencv.ColorRange.GREEN);
                telemetry.addLine("Target color: GREEN");
            }
            if (gamepad1.right_bumper) {
                camera.setTargetColor(org.firstinspires.ftc.vision.opencv.ColorRange.YELLOW);
                telemetry.addLine("Target color: YELLOW");
            }
        }
    }
    
    private void handleExposureControls() {
        // Auto exposure toggle
        if (gamepad1.dpad_left) {
            AUTO_EXPOSURE = true;
            camera.setAutoExposure(true);
            telemetry.addLine("Auto exposure: ENABLED");
        }
        
        if (gamepad1.dpad_right) {
            AUTO_EXPOSURE = false;
            camera.setAutoExposure(false);
            camera.setCameraExposure(MANUAL_EXPOSURE_MS, MANUAL_GAIN);
            telemetry.addLine("Manual exposure: ENABLED");
        }
        
        // Manual exposure adjustments (when auto exposure is off)
        if (!AUTO_EXPOSURE) {
            if (gamepad1.left_trigger > 0.5) {
                MANUAL_EXPOSURE_MS = Math.max(MANUAL_EXPOSURE_MS - 1, 1);
                camera.setCameraExposure(MANUAL_EXPOSURE_MS, MANUAL_GAIN);
            }
            
            if (gamepad1.right_trigger > 0.5) {
                MANUAL_EXPOSURE_MS = Math.min(MANUAL_EXPOSURE_MS + 1, 50);
                camera.setCameraExposure(MANUAL_EXPOSURE_MS, MANUAL_GAIN);
            }
            
            if (gamepad1.left_stick_button) {
                MANUAL_GAIN = Math.max(MANUAL_GAIN - 25, 0);
                camera.setCameraExposure(MANUAL_EXPOSURE_MS, MANUAL_GAIN);
            }
            
            if (gamepad1.right_stick_button) {
                MANUAL_GAIN = Math.min(MANUAL_GAIN + 25, 500);
                camera.setCameraExposure(MANUAL_EXPOSURE_MS, MANUAL_GAIN);
            }
        }
    }
    
    private void displayPerformanceInfo() {
        // Camera state information
        telemetry.addData("Performance", "");
        telemetry.addData("  Camera State", "Streaming"); // Would get actual state from VisionPortal
        telemetry.addData("  Frame Rate", "%d FPS", STREAM_FPS);
        
        // Memory usage (approximate)
        Runtime runtime = Runtime.getRuntime();
        long totalMemory = runtime.totalMemory() / 1024 / 1024; // MB
        long freeMemory = runtime.freeMemory() / 1024 / 1024;   // MB
        long usedMemory = totalMemory - freeMemory;
        
        telemetry.addData("  Memory Usage", "%d MB / %d MB", usedMemory, totalMemory);
        
        // Processing load indicators
        if (SHOW_APRILTAGS && SHOW_COLOR_DETECTION) {
            telemetry.addData("  Processing Load", "HIGH (AprilTag + Color)");
        } else if (SHOW_APRILTAGS || SHOW_COLOR_DETECTION) {
            telemetry.addData("  Processing Load", "MEDIUM");
        } else {
            telemetry.addData("  Processing Load", "LOW (Stream Only)");
        }
        
        telemetry.addLine();
    }
    
    private void displayControls() {
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Camera:");
        telemetry.addLine("• START: Start streaming");
        telemetry.addLine("• BACK: Stop streaming");
        telemetry.addLine("• D-pad up/down: Adjust FPS");
        telemetry.addLine();
        
        telemetry.addLine("Processing:");
        telemetry.addLine("• A: Toggle AprilTag detection");
        telemetry.addLine("• B: Toggle color detection");
        
        if (SHOW_COLOR_DETECTION) {
            telemetry.addLine("• X: Target RED");
            telemetry.addLine("• Y: Target BLUE");
            telemetry.addLine("• LB: Target GREEN");
            telemetry.addLine("• RB: Target YELLOW");
        }
        
        telemetry.addLine();
        telemetry.addLine("Exposure:");
        telemetry.addLine("• D-pad left: Auto exposure");
        telemetry.addLine("• D-pad right: Manual exposure");
        
        if (!AUTO_EXPOSURE) {
            telemetry.addLine("• Left trigger: Decrease exposure");
            telemetry.addLine("• Right trigger: Increase exposure");
            telemetry.addLine("• Left stick click: Decrease gain");
            telemetry.addLine("• Right stick click: Increase gain");
        }
        
        telemetry.addLine();
        telemetry.addLine("Dashboard:");
        telemetry.addLine("• Select 'Camera' view in dashboard");
        telemetry.addLine("• Adjust dashboard settings as needed");
    }
}
