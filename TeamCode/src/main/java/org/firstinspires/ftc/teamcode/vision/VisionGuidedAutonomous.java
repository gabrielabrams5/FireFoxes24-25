package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.vision.utils.VisionNavigation;
import org.firstinspires.ftc.teamcode.vision.utils.VisionMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

@Config
@Autonomous(name = "Vision Guided Autonomous", group = "Vision")
public class VisionGuidedAutonomous extends LinearOpMode {
    
    // Tunable parameters
    public static double APPROACH_DISTANCE = 12.0; // inches from AprilTag
    public static double ALIGNMENT_TOLERANCE = 2.0; // degrees
    public static double DISTANCE_TOLERANCE = 1.0; // inches
    public static double MAX_APPROACH_SPEED = 0.4;
    public static double MAX_SEARCH_TIME = 5.0; // seconds
    public static int TARGET_APRILTAG_ID = 1;
    public static ColorRange SAMPLE_COLOR = ColorRange.YELLOW;
    
    // Starting position
    public static double START_X = 12;
    public static double START_Y = -60;
    public static double START_HEADING = 90;
    
    private CameraSubsystem camera;
    private MecanumDrive drive;
    private VisionNavigation.VisionPIDController headingController;
    private VisionNavigation.VisionPIDController distanceController;
    
    @Override
    public void runOpMode() {
        // Initialize drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING)));
        
        // Initialize camera
        camera = new CameraSubsystem(hardwareMap, telemetry);
        camera.setAprilTagEnabled(true);
        camera.setColorDetectionEnabled(true);
        camera.setTargetColor(SAMPLE_COLOR);
        
        // Initialize PID controllers for vision guidance
        headingController = new VisionNavigation.VisionPIDController(0.02, 0.001, 0.005);
        distanceController = new VisionNavigation.VisionPIDController(0.03, 0.001, 0.01);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target AprilTag", TARGET_APRILTAG_ID);
        telemetry.addData("Sample Color", SAMPLE_COLOR.toString());
        telemetry.addData("Starting Position", "X=%.1f, Y=%.1f, H=%.0f°", START_X, START_Y, START_HEADING);
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Execute vision-guided autonomous sequence
            Actions.runBlocking(
                new SequentialAction(
                    // Move to search position
                    drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(24, -36))
                        .build(),
                    
                    // Search for and approach AprilTag
                    new SearchAndApproachAprilTag(),
                    
                    // Search for and collect sample
                    new SearchAndCollectSample(),
                    
                    // Return to scoring position
                    drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(48, -48))
                        .build(),
                    
                    // Score sample (placeholder)
                    new SleepAction(1.0)
                )
            );
        }
        
        // Cleanup
        camera.close();
    }
    
    /**
     * Action to search for and approach a specific AprilTag
     */
    public class SearchAndApproachAprilTag implements Action {
        private boolean initialized = false;
        private long searchStartTime;
        private AprilTagDetection targetTag = null;
        
        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                searchStartTime = System.currentTimeMillis();
                headingController.reset();
                distanceController.reset();
                initialized = true;
                
                packet.put("Vision Action", "Searching for AprilTag " + TARGET_APRILTAG_ID);
            }
            
            // Update camera detections
            camera.updateAprilTagDetections();
            List<AprilTagDetection> detections = camera.getAprilTagDetections();
            
            // Check for timeout
            if ((System.currentTimeMillis() - searchStartTime) / 1000.0 > MAX_SEARCH_TIME) {
                packet.put("Vision Status", "AprilTag search timeout");
                return false; // End action
            }
            
            // Find target tag
            targetTag = null;
            if (detections != null) {
                for (AprilTagDetection detection : detections) {
                    if (detection.id == TARGET_APRILTAG_ID) {
                        targetTag = detection;
                        break;
                    }
                }
            }
            
            if (targetTag == null) {
                // Slowly rotate to search for tag
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.2));
                packet.put("Vision Status", "Searching for AprilTag...");
                return true; // Continue searching
            }
            
            // Calculate approach trajectory
            VisionNavigation.ApproachTrajectory trajectory = 
                VisionNavigation.calculateApproachTrajectory(
                    targetTag, APPROACH_DISTANCE, MAX_APPROACH_SPEED, 
                    ALIGNMENT_TOLERANCE, DISTANCE_TOLERANCE);
            
            packet.put("Vision Status", "Approaching AprilTag");
            packet.put("Distance", camera.getDistanceToTarget(targetTag));
            packet.put("Bearing", Math.toDegrees(camera.getHeadingToTarget(targetTag)));
            packet.put("Aligned", trajectory.isAligned);
            packet.put("At Target", trajectory.isAtTarget);
            
            if (trajectory.isAligned && trajectory.isAtTarget) {
                // Successfully reached target
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                packet.put("Vision Status", "AprilTag approach complete");
                return false; // End action
            }
            
            // Apply calculated movements
            drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(trajectory.driveX, trajectory.driveY), 
                trajectory.rotate));
            
            return true; // Continue approaching
        }
    }
    
    /**
     * Action to search for and collect a color sample
     */
    public class SearchAndCollectSample implements Action {
        private boolean initialized = false;
        private long searchStartTime;
        private ColorBlobLocatorProcessor.Blob targetBlob = null;
        
        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                searchStartTime = System.currentTimeMillis();
                initialized = true;
                packet.put("Vision Action", "Searching for " + SAMPLE_COLOR + " sample");
            }
            
            // Update camera detections
            camera.updateColorDetections();
            List<ColorBlobLocatorProcessor.Blob> blobs = camera.getColorBlobs();
            
            // Check for timeout
            if ((System.currentTimeMillis() - searchStartTime) / 1000.0 > MAX_SEARCH_TIME) {
                packet.put("Vision Status", "Sample search timeout");
                return false;
            }
            
            // Filter and find best blob
            if (blobs != null && !blobs.isEmpty()) {
                // Filter by area to ignore noise
                blobs = VisionMath.ColorBlobFilter.filterByArea(blobs, 100, 5000);
                // Get the largest blob
                targetBlob = camera.getLargestBlob();
            } else {
                targetBlob = null;
            }
            
            if (targetBlob == null) {
                // Slowly search for sample
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.1, 0), 0.1));
                packet.put("Vision Status", "Searching for sample...");
                return true;
            }
            
            // Calculate approach trajectory for blob
            double targetArea = 1500.0; // Desired blob size when close enough
            VisionNavigation.ApproachTrajectory trajectory = 
                VisionNavigation.calculateBlobApproachTrajectory(
                    targetBlob, CameraSubsystem.CAMERA_WIDTH, targetArea, 
                    MAX_APPROACH_SPEED, ALIGNMENT_TOLERANCE, 300);
            
            packet.put("Vision Status", "Approaching sample");
            packet.put("Blob Area", targetBlob.getContourArea());
            packet.put("Blob Center X", camera.getBlobCenterX(targetBlob));
            packet.put("Angle from Center", camera.getBlobAngleFromCenter(targetBlob));
            packet.put("Aligned", trajectory.isAligned);
            packet.put("At Target", trajectory.isAtTarget);
            
            if (trajectory.isAligned && trajectory.isAtTarget) {
                // Close enough to collect sample
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                
                // Add sample collection actions here
                // Example: intake.collect(), wait, etc.
                
                packet.put("Vision Status", "Sample collection complete");
                return false;
            }
            
            // Apply calculated movements
            drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(trajectory.driveX, trajectory.driveY), 
                trajectory.rotate));
            
            return true;
        }
    }
    
    /**
     * Action to perform vision-guided precision alignment
     */
    public class VisionAlignmentAction implements Action {
        private final int targetAprilTagId;
        private final double targetDistance;
        private final double toleranceAngle;
        private final double toleranceDist;
        private boolean initialized = false;
        
        public VisionAlignmentAction(int tagId, double distance, double angleToleranceDeg, double distTolerance) {
            this.targetAprilTagId = tagId;
            this.targetDistance = distance;
            this.toleranceAngle = angleToleranceDeg;
            this.toleranceDist = distTolerance;
        }
        
        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                headingController.reset();
                distanceController.reset();
                initialized = true;
            }
            
            // Update detections
            camera.updateAprilTagDetections();
            AprilTagDetection target = camera.getTargetAprilTag(targetAprilTagId);
            
            if (target == null) {
                packet.put("Alignment Status", "Target not found");
                return false;
            }
            
            // Calculate errors
            double currentDistance = camera.getDistanceToTarget(target);
            double bearing = Math.toDegrees(camera.getHeadingToTarget(target));
            
            double distanceError = currentDistance - targetDistance;
            double angleError = bearing;
            
            // Check if aligned
            if (Math.abs(angleError) < toleranceAngle && Math.abs(distanceError) < toleranceDist) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                packet.put("Alignment Status", "Complete");
                return false;
            }
            
            // Calculate control outputs
            double turnPower = headingController.calculate(Math.toRadians(angleError));
            double drivePower = distanceController.calculate(distanceError);
            
            // Limit power
            turnPower = Math.max(-MAX_APPROACH_SPEED, Math.min(MAX_APPROACH_SPEED, turnPower));
            drivePower = Math.max(-MAX_APPROACH_SPEED, Math.min(MAX_APPROACH_SPEED, drivePower));
            
            // Apply powers
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -drivePower), turnPower));
            
            packet.put("Alignment Status", "Aligning");
            packet.put("Distance Error", distanceError);
            packet.put("Angle Error", angleError);
            packet.put("Turn Power", turnPower);
            packet.put("Drive Power", drivePower);
            
            return true;
        }
    }
    
    /**
     * Utility method to create a vision search pattern
     */
    private Action createSearchPattern() {
        return new SequentialAction(
            // Search pattern: sweep left and right
            drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(30))
                .build(),
            new SleepAction(0.5),
            drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(-60))
                .build(),
            new SleepAction(0.5),
            drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(30))
                .build()
        );
    }
}
