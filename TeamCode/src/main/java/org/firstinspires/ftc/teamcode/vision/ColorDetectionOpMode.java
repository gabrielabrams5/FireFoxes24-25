package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

@Config
@TeleOp(name = "Color Detection", group = "Vision")
public class ColorDetectionOpMode extends LinearOpMode {
    
    // Tunable parameters
    public static ColorRange TARGET_COLOR = ColorRange.BLUE;
    public static boolean TRACK_LARGEST = true;
    public static double MIN_BLOB_AREA = 100.0;
    public static double ALIGNMENT_TOLERANCE = 5.0; // degrees
    public static double DRIVE_SPEED = 0.3;
    public static double TURN_SPEED = 0.2;
    public static boolean AUTO_TRACK = false;
    
    private CameraSubsystem camera;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorRange currentTargetColor = ColorRange.BLUE;
    
    @Override
    public void runOpMode() {
        // Initialize telemetry for dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize camera
        camera = new CameraSubsystem(hardwareMap, telemetry);
        camera.setAprilTagEnabled(false);
        camera.setColorDetectionEnabled(true);
        camera.setTargetColor(TARGET_COLOR);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press start to begin color detection");
        telemetry.addLine("Use gamepad for controls:");
        telemetry.addLine("• Left stick: Drive");
        telemetry.addLine("• Right stick: Turn");
        telemetry.addLine("• A/B/X/Y: Change target color");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            // Update camera detections
            camera.updateColorDetections();
            
            // Get color blob detections
            List<ColorBlobLocatorProcessor.Blob> blobs = camera.getColorBlobs();
            
            // Display general information
            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.addData("Target Color", currentTargetColor.toString());
            telemetry.addData("Blobs Detected", blobs != null ? blobs.size() : 0);
            telemetry.addLine();
            
            if (blobs != null && !blobs.isEmpty()) {
                if (TRACK_LARGEST) {
                    // Track the largest blob
                    ColorBlobLocatorProcessor.Blob largestBlob = camera.getLargestBlob();
                    if (largestBlob != null) {
                        displayBlobInfo("LARGEST", largestBlob);
                        
                        if (AUTO_TRACK) {
                            performAutoTracking(largestBlob);
                        }
                    }
                } else {
                    // Show all blobs
                    for (int i = 0; i < Math.min(blobs.size(), 5); i++) { // Limit to 5 for telemetry
                        ColorBlobLocatorProcessor.Blob blob = blobs.get(i);
                        displayBlobInfo("BLOB " + (i + 1), blob);
                    }
                    
                    if (blobs.size() > 5) {
                        telemetry.addLine(String.format("... and %d more blobs", blobs.size() - 5));
                    }
                }
                
                // Show closest blob info
                ColorBlobLocatorProcessor.Blob closestBlob = camera.getClosestBlob();
                if (closestBlob != null && (!TRACK_LARGEST || closestBlob != camera.getLargestBlob())) {
                    displayBlobInfo("CLOSEST", closestBlob);
                }
                
            } else {
                telemetry.addLine("No color blobs detected");
                telemetry.addLine();
                telemetry.addLine("Try:");
                telemetry.addLine("• Adjusting lighting");
                telemetry.addLine("• Changing target color");
                telemetry.addLine("• Moving camera closer");
                telemetry.addLine("• Checking color settings in dashboard");
            }
            
            // Handle controls
            handleColorControls();
            handleManualDriving();
            handleCameraControls();
            
            // Update camera telemetry
            camera.updateTelemetry();
            
            telemetry.update();
        }
        
        // Cleanup
        camera.close();
    }
    
    private void displayBlobInfo(String label, ColorBlobLocatorProcessor.Blob blob) {
        if (blob == null) return;
        
        telemetry.addLine(String.format("%s BLOB:", label));
        telemetry.addData("  Area", "%.0f pixels", blob.getContourArea());
        telemetry.addData("  Center", "(%.0f, %.0f)", 
            camera.getBlobCenterX(blob), camera.getBlobCenterY(blob));
        telemetry.addData("  Angle from Center", "%.1f degrees", 
            camera.getBlobAngleFromCenter(blob));
        
        // Box fit information
        if (blob.getBoxFit() != null) {
            telemetry.addData("  Box Size", "%.0f x %.0f pixels",
                blob.getBoxFit().size.width, blob.getBoxFit().size.height);
            telemetry.addData("  Box Angle", "%.1f degrees", blob.getBoxFit().angle);
        }
        
        // Density and shape metrics
        double density = blob.getDensity();
        double aspectRatio = blob.getAspectRatio();
        
        telemetry.addData("  Density", "%.2f", density);
        telemetry.addData("  Aspect Ratio", "%.2f", aspectRatio);
        
        // Alignment status
        double angleFromCenter = camera.getBlobAngleFromCenter(blob);
        boolean aligned = Math.abs(angleFromCenter) < ALIGNMENT_TOLERANCE;
        telemetry.addData("  Aligned", aligned ? "✓ YES" : "✗ NO");
        
        telemetry.addLine();
    }
    
    private void performAutoTracking(ColorBlobLocatorProcessor.Blob targetBlob) {
        if (targetBlob == null) return;
        
        double angleError = camera.getBlobAngleFromCenter(targetBlob);
        double blobArea = targetBlob.getContourArea();
        
        double driveY = 0;
        double turn = 0;
        
        // Calculate turning to center the blob
        if (Math.abs(angleError) > ALIGNMENT_TOLERANCE) {
            turn = angleError * 0.02; // Proportional control
            turn = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, turn));
        }
        
        // Drive forward/backward based on blob size (bigger = closer)
        double targetArea = 2000.0; // Target blob area
        double areaError = targetArea - blobArea;
        if (Math.abs(areaError) > 500) {
            driveY = Math.signum(areaError) * DRIVE_SPEED * 0.5;
            driveY = Math.max(-DRIVE_SPEED, Math.min(DRIVE_SPEED, driveY));
        }
        
        telemetry.addData("Auto Track", "Drive: %.2f, Turn: %.2f", driveY, turn);
        telemetry.addData("Angle Error", "%.1f degrees", angleError);
        telemetry.addData("Area Error", "%.0f pixels", areaError);
        
        // Note: Actual motor control would be implemented here
        // Example: drive.drive(0, driveY, turn);
    }
    
    private void handleColorControls() {
        // Color selection with gamepad buttons
        if (gamepad1.a && currentTargetColor != ColorRange.RED) {
            currentTargetColor = ColorRange.RED;
            camera.setTargetColor(ColorRange.RED);
            TARGET_COLOR = ColorRange.RED;
        }
        
        if (gamepad1.b && currentTargetColor != ColorRange.BLUE) {
            currentTargetColor = ColorRange.BLUE;
            camera.setTargetColor(ColorRange.BLUE);
            TARGET_COLOR = ColorRange.BLUE;
        }
        
        if (gamepad1.x && currentTargetColor != ColorRange.GREEN) {
            currentTargetColor = ColorRange.GREEN;
            camera.setTargetColor(ColorRange.GREEN);
            TARGET_COLOR = ColorRange.GREEN;
        }
        
        if (gamepad1.y && currentTargetColor != ColorRange.YELLOW) {
            currentTargetColor = ColorRange.YELLOW;
            camera.setTargetColor(ColorRange.YELLOW);
            TARGET_COLOR = ColorRange.YELLOW;
        }
        
        // Toggle tracking mode
        if (gamepad1.left_bumper) {
            TRACK_LARGEST = !TRACK_LARGEST;
        }
        
        // Toggle auto tracking
        if (gamepad1.right_bumper) {
            AUTO_TRACK = !AUTO_TRACK;
        }
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
        // Camera exposure controls
        if (gamepad1.dpad_up) {
            camera.setAutoExposure(false);
            camera.setCameraExposure(6, 250); // Low exposure for bright conditions
        }
        
        if (gamepad1.dpad_down) {
            camera.setAutoExposure(false);
            camera.setCameraExposure(20, 250); // Higher exposure for darker conditions
        }
        
        if (gamepad1.dpad_left) {
            camera.setAutoExposure(true);
        }
        
        if (gamepad1.dpad_right) {
            // Reset to default manual settings
            camera.setAutoExposure(false);
            camera.setCameraExposure(CameraSubsystem.EXPOSURE_MS, CameraSubsystem.GAIN);
        }
        
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("• Left stick: Drive/Strafe");
        telemetry.addLine("• Right stick: Turn");
        telemetry.addLine("• A: Target RED");
        telemetry.addLine("• B: Target BLUE");
        telemetry.addLine("• X: Target GREEN");
        telemetry.addLine("• Y: Target YELLOW");
        telemetry.addLine("• Left bumper: Toggle track mode");
        telemetry.addLine("• Right bumper: Toggle auto track");
        telemetry.addLine("• D-pad up: Low exposure");
        telemetry.addLine("• D-pad down: High exposure");
        telemetry.addLine("• D-pad left: Auto exposure");
        telemetry.addLine("• D-pad right: Default exposure");
        telemetry.addLine();
        telemetry.addData("Current Mode", "Track %s, Auto Track: %s",
            TRACK_LARGEST ? "LARGEST" : "ALL", AUTO_TRACK ? "ON" : "OFF");
    }
}
