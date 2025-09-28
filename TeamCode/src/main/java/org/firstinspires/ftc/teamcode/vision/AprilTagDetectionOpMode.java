package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@TeleOp(name = "AprilTag Detection", group = "Vision")
public class AprilTagDetectionOpMode extends LinearOpMode {
    
    // Tunable parameters
    public static int TARGET_TAG_ID = 1; // Change this to target specific tags
    public static boolean SHOW_ALL_TAGS = true;
    public static boolean AUTO_ALIGN = false;
    public static double DRIVE_SPEED = 0.3;
    public static double TURN_SPEED = 0.2;
    public static double ALIGNMENT_TOLERANCE = 2.0; // degrees
    public static double DISTANCE_TOLERANCE = 2.0; // inches
    
    private CameraSubsystem camera;
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        // Initialize telemetry for dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize camera
        camera = new CameraSubsystem(hardwareMap, telemetry);
        camera.setAprilTagEnabled(true);
        camera.setColorDetectionEnabled(false);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press start to begin AprilTag detection");
        telemetry.addLine("Use gamepad for manual driving");
        telemetry.addLine("Set AUTO_ALIGN to true for automatic alignment");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            // Update camera detections
            camera.updateAprilTagDetections();
            
            // Get all detections
            List<AprilTagDetection> currentDetections = camera.getAprilTagDetections();
            
            // Display general information
            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.addData("Tags Detected", currentDetections != null ? currentDetections.size() : 0);
            telemetry.addLine();
            
            if (currentDetections != null && !currentDetections.isEmpty()) {
                if (SHOW_ALL_TAGS) {
                    // Show all detected tags
                    for (AprilTagDetection detection : currentDetections) {
                        displayTagInfo(detection);
                    }
                } else {
                    // Show only target tag
                    AprilTagDetection targetTag = camera.getTargetAprilTag(TARGET_TAG_ID);
                    if (targetTag != null) {
                        displayTagInfo(targetTag);
                        
                        if (AUTO_ALIGN) {
                            performAutoAlignment(targetTag);
                        }
                    } else {
                        telemetry.addLine(String.format("Target tag %d not found", TARGET_TAG_ID));
                    }
                }
            } else {
                telemetry.addLine("No AprilTags detected");
                telemetry.addLine();
                telemetry.addLine("Make sure:");
                telemetry.addLine("• AprilTags are in view");
                telemetry.addLine("• Lighting is adequate");
                telemetry.addLine("• Camera is focused");
            }
            
            // Manual driving controls
            handleManualDriving();
            
            // Camera controls
            handleCameraControls();
            
            // Update camera telemetry
            camera.updateTelemetry();
            
            telemetry.update();
        }
        
        // Cleanup
        camera.close();
    }
    
    private void displayTagInfo(AprilTagDetection detection) {
        telemetry.addLine(String.format("Tag ID %d (%s)", 
            detection.id, 
            detection.metadata != null ? detection.metadata.name : "Unknown"));
        
        if (detection.ftcPose != null) {
            telemetry.addData("  Position", "X=%.1f, Y=%.1f, Z=%.1f (inches)",
                detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
            telemetry.addData("  Rotation", "Roll=%.1f, Pitch=%.1f, Yaw=%.1f (degrees)",
                detection.ftcPose.roll, detection.ftcPose.pitch, detection.ftcPose.yaw);
            
            // Calculate distance and bearing
            double distance = camera.getDistanceToTarget(detection);
            double bearing = Math.toDegrees(camera.getHeadingToTarget(detection));
            
            telemetry.addData("  Range & Bearing", "%.1f inches, %.1f degrees", distance, bearing);
            
            // Alignment status
            boolean alignedAngle = Math.abs(bearing) < ALIGNMENT_TOLERANCE;
            boolean alignedDistance = Math.abs(distance - 12) < DISTANCE_TOLERANCE; // Target 12 inches
            
            telemetry.addData("  Alignment", "Angle: %s, Distance: %s",
                alignedAngle ? "✓" : "✗",
                alignedDistance ? "✓" : "✗");
        } else {
            telemetry.addLine("  No pose information available");
        }
        
        telemetry.addLine();
    }
    
    private void performAutoAlignment(AprilTagDetection targetTag) {
        if (targetTag.ftcPose == null) return;
        
        double distance = camera.getDistanceToTarget(targetTag);
        double bearing = Math.toDegrees(camera.getHeadingToTarget(targetTag));
        
        double driveY = 0;
        double driveX = 0;
        double turn = 0;
        
        // Calculate turning adjustment
        if (Math.abs(bearing) > ALIGNMENT_TOLERANCE) {
            turn = Math.signum(bearing) * TURN_SPEED;
            turn = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, turn));
        }
        
        // Calculate forward/backward adjustment (target 12 inches)
        double targetDistance = 12.0;
        double distanceError = distance - targetDistance;
        if (Math.abs(distanceError) > DISTANCE_TOLERANCE) {
            driveY = -Math.signum(distanceError) * DRIVE_SPEED;
            driveY = Math.max(-DRIVE_SPEED, Math.min(DRIVE_SPEED, driveY));
        }
        
        // Apply movements (would need drive subsystem integration)
        telemetry.addData("Auto Align", "Drive: %.2f, Turn: %.2f", driveY, turn);
        telemetry.addData("Bearing Error", "%.1f degrees", bearing);
        telemetry.addData("Distance Error", "%.1f inches", distanceError);
        
        // Note: Actual motor control would be implemented here
        // Example: drive.drive(driveX, driveY, turn);
    }
    
    private void handleManualDriving() {
        // Manual driving with gamepad
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = gamepad1.left_stick_x * DRIVE_SPEED;
        double turn = gamepad1.right_stick_x * TURN_SPEED;
        
        if (Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05) {
            telemetry.addData("Manual Control", "Drive: %.2f, Strafe: %.2f, Turn: %.2f", 
                drive, strafe, turn);
            
            // Note: Actual motor control would be implemented here
            // Example: drive.drive(strafe, drive, turn);
        }
    }
    
    private void handleCameraControls() {
        // Camera control with gamepad buttons
        if (gamepad1.a) {
            camera.setAutoExposure(true);
            telemetry.addLine("Auto exposure enabled");
        }
        
        if (gamepad1.b) {
            camera.setAutoExposure(false);
            telemetry.addLine("Manual exposure enabled");
        }
        
        if (gamepad1.x) {
            camera.stopStreaming();
            telemetry.addLine("Camera streaming stopped");
        }
        
        if (gamepad1.y) {
            camera.startStreaming();
            telemetry.addLine("Camera streaming started");
        }
        
        // Target tag selection
        if (gamepad1.dpad_up) {
            TARGET_TAG_ID = Math.min(TARGET_TAG_ID + 1, 30);
        }
        if (gamepad1.dpad_down) {
            TARGET_TAG_ID = Math.max(TARGET_TAG_ID - 1, 1);
        }
        
        // Toggle show all tags
        if (gamepad1.dpad_left) {
            SHOW_ALL_TAGS = !SHOW_ALL_TAGS;
        }
        
        // Toggle auto align
        if (gamepad1.dpad_right) {
            AUTO_ALIGN = !AUTO_ALIGN;
        }
        
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("• Left stick: Drive");
        telemetry.addLine("• Right stick: Turn");
        telemetry.addLine("• A: Auto exposure ON");
        telemetry.addLine("• B: Manual exposure");
        telemetry.addLine("• X: Stop camera");
        telemetry.addLine("• Y: Start camera");
        telemetry.addLine("• D-pad up/down: Change target tag");
        telemetry.addLine("• D-pad left: Toggle show all tags");
        telemetry.addLine("• D-pad right: Toggle auto align");
        telemetry.addLine();
        telemetry.addData("Current Settings", "Target ID: %d, Show All: %s, Auto Align: %s",
            TARGET_TAG_ID, SHOW_ALL_TAGS ? "ON" : "OFF", AUTO_ALIGN ? "ON" : "OFF");
    }
}
