package org.firstinspires.ftc.teamcode.vision.utils;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

/**
 * Utility class for vision-based navigation and positioning calculations
 */
public class VisionNavigation {
    
    // Constants for field navigation
    public static final double FIELD_WIDTH = 144.0; // inches
    public static final double FIELD_HEIGHT = 144.0; // inches
    
    // Camera calibration constants (adjust for your specific camera)
    public static final double CAMERA_HEIGHT = 8.0; // inches off ground
    public static final double CAMERA_FORWARD_OFFSET = 6.0; // inches from robot center
    public static final double HORIZONTAL_FOV = 78.0; // degrees
    public static final double VERTICAL_FOV = 58.0; // degrees
    
    /**
     * Calculate robot pose relative to an AprilTag
     */
    public static Pose2d calculateRobotPoseFromTag(AprilTagDetection tag, Pose2d knownTagPose) {
        if (tag == null || tag.ftcPose == null || knownTagPose == null) {
            return null;
        }
        
        // Convert tag-relative coordinates to field coordinates
        double tagRelativeX = tag.ftcPose.x;
        double tagRelativeY = tag.ftcPose.y;
        double tagRelativeZ = tag.ftcPose.z;
        double tagRelativeYaw = Math.toRadians(tag.ftcPose.yaw);
        
        // Account for camera offset from robot center
        double robotX = tagRelativeX - CAMERA_FORWARD_OFFSET * Math.cos(tagRelativeYaw);
        double robotY = tagRelativeY - CAMERA_FORWARD_OFFSET * Math.sin(tagRelativeYaw);
        
        // Transform to field coordinates
        double fieldX = knownTagPose.position.x - robotX;
        double fieldY = knownTagPose.position.y - robotY;
        double fieldHeading = knownTagPose.heading.toDouble() + tagRelativeYaw;
        
        return new Pose2d(fieldX, fieldY, fieldHeading);
    }
    
    /**
     * Calculate heading correction needed to align with target
     */
    public static double calculateHeadingCorrection(AprilTagDetection target, double targetHeading) {
        if (target == null || target.ftcPose == null) {
            return 0;
        }
        
        double currentHeading = Math.toRadians(target.ftcPose.yaw);
        double headingError = targetHeading - currentHeading;
        
        // Normalize to [-π, π]
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        
        return headingError;
    }
    
    /**
     * Calculate distance to target in the horizontal plane
     */
    public static double calculateHorizontalDistance(AprilTagDetection target) {
        if (target == null || target.ftcPose == null) {
            return Double.MAX_VALUE;
        }
        
        return Math.sqrt(
            Math.pow(target.ftcPose.x, 2) + 
            Math.pow(target.ftcPose.z, 2)
        );
    }
    
    /**
     * Calculate the angle to turn to face a target
     */
    public static double calculateBearingToTarget(AprilTagDetection target) {
        if (target == null || target.ftcPose == null) {
            return 0;
        }
        
        return Math.atan2(target.ftcPose.x, target.ftcPose.z);
    }
    
    /**
     * Estimate target distance from blob size (requires calibration)
     */
    public static double estimateDistanceFromBlobSize(ColorBlobLocatorProcessor.Blob blob, 
                                                     double knownObjectWidth, double knownDistance, double knownBlobWidth) {
        if (blob == null) {
            return Double.MAX_VALUE;
        }
        
        double currentBlobWidth = blob.getBoxFit().size.width;
        
        // Use inverse square law approximation
        return knownDistance * Math.sqrt(knownBlobWidth / currentBlobWidth);
    }
    
    /**
     * Convert pixel coordinates to field angles
     */
    public static class PixelToAngle {
        private final int imageWidth;
        private final int imageHeight;
        private final double horizontalFOV;
        private final double verticalFOV;
        
        public PixelToAngle(int imageWidth, int imageHeight, double horizontalFOV, double verticalFOV) {
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.horizontalFOV = horizontalFOV;
            this.verticalFOV = verticalFOV;
        }
        
        public double getHorizontalAngle(double pixelX) {
            double centerX = imageWidth / 2.0;
            double offsetX = pixelX - centerX;
            double degreesPerPixel = horizontalFOV / imageWidth;
            return offsetX * degreesPerPixel;
        }
        
        public double getVerticalAngle(double pixelY) {
            double centerY = imageHeight / 2.0;
            double offsetY = centerY - pixelY; // Y increases downward in image
            double degreesPerPixel = verticalFOV / imageHeight;
            return offsetY * degreesPerPixel;
        }
    }
    
    /**
     * PID controller for vision-based alignment
     */
    public static class VisionPIDController {
        private double kP, kI, kD;
        private double previousError = 0;
        private double integralSum = 0;
        private double maxIntegral = 1.0;
        private long lastUpdateTime = 0;
        
        public VisionPIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
        
        public double calculate(double error) {
            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastUpdateTime) / 1e9; // Convert to seconds
            
            if (lastUpdateTime == 0) {
                deltaTime = 0.02; // Default 20ms for first iteration
            }
            
            // Proportional term
            double proportional = kP * error;
            
            // Integral term
            integralSum += error * deltaTime;
            integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
            double integral = kI * integralSum;
            
            // Derivative term
            double derivative = kD * (error - previousError) / deltaTime;
            
            // Update for next iteration
            previousError = error;
            lastUpdateTime = currentTime;
            
            return proportional + integral + derivative;
        }
        
        public void reset() {
            previousError = 0;
            integralSum = 0;
            lastUpdateTime = 0;
        }
        
        public void setMaxIntegral(double maxIntegral) {
            this.maxIntegral = maxIntegral;
        }
    }
    
    /**
     * Calculate trajectory to approach a target safely
     */
    public static class ApproachTrajectory {
        public final double driveX;
        public final double driveY;
        public final double rotate;
        public final boolean isAligned;
        public final boolean isAtTarget;
        
        public ApproachTrajectory(double driveX, double driveY, double rotate, 
                                boolean isAligned, boolean isAtTarget) {
            this.driveX = driveX;
            this.driveY = driveY;
            this.rotate = rotate;
            this.isAligned = isAligned;
            this.isAtTarget = isAtTarget;
        }
    }
    
    /**
     * Calculate approach trajectory for AprilTag
     */
    public static ApproachTrajectory calculateApproachTrajectory(AprilTagDetection target,
                                                               double targetDistance,
                                                               double maxSpeed,
                                                               double alignmentTolerance,
                                                               double distanceTolerance) {
        if (target == null || target.ftcPose == null) {
            return new ApproachTrajectory(0, 0, 0, false, false);
        }
        
        double currentDistance = calculateHorizontalDistance(target);
        double bearing = calculateBearingToTarget(target);
        
        // Check if aligned and at target
        boolean isAligned = Math.abs(Math.toDegrees(bearing)) < alignmentTolerance;
        boolean isAtTarget = Math.abs(currentDistance - targetDistance) < distanceTolerance;
        
        // Calculate rotation
        double rotate = 0;
        if (!isAligned) {
            rotate = Math.signum(bearing) * maxSpeed * 0.5; // Slower rotation
            rotate = Math.max(-maxSpeed, Math.min(maxSpeed, rotate));
        }
        
        // Calculate forward/backward movement
        double driveY = 0;
        if (isAligned && !isAtTarget) {
            double distanceError = currentDistance - targetDistance;
            driveY = -Math.signum(distanceError) * maxSpeed * 0.7;
            driveY = Math.max(-maxSpeed, Math.min(maxSpeed, driveY));
        }
        
        return new ApproachTrajectory(0, driveY, rotate, isAligned, isAtTarget);
    }
    
    /**
     * Calculate approach trajectory for color blob
     */
    public static ApproachTrajectory calculateBlobApproachTrajectory(ColorBlobLocatorProcessor.Blob target,
                                                                   int imageWidth,
                                                                   double targetArea,
                                                                   double maxSpeed,
                                                                   double alignmentTolerance,
                                                                   double areaTolerance) {
        if (target == null) {
            return new ApproachTrajectory(0, 0, 0, false, false);
        }
        
        double blobCenterX = target.getBoxFit().center.x;
        double imageCenterX = imageWidth / 2.0;
        double offsetPixels = blobCenterX - imageCenterX;
        double currentArea = target.getContourArea();
        
        // Convert pixel offset to angle
        double degreesPerPixel = HORIZONTAL_FOV / imageWidth;
        double angleError = offsetPixels * degreesPerPixel;
        
        // Check alignment and target size
        boolean isAligned = Math.abs(angleError) < alignmentTolerance;
        boolean isAtTarget = Math.abs(currentArea - targetArea) < areaTolerance;
        
        // Calculate rotation
        double rotate = 0;
        if (!isAligned) {
            rotate = angleError * 0.02; // Proportional control
            rotate = Math.max(-maxSpeed * 0.5, Math.min(maxSpeed * 0.5, rotate));
        }
        
        // Calculate forward/backward movement based on blob size
        double driveY = 0;
        if (isAligned && !isAtTarget) {
            double areaError = targetArea - currentArea;
            driveY = Math.signum(areaError) * maxSpeed * 0.6;
            driveY = Math.max(-maxSpeed, Math.min(maxSpeed, driveY));
        }
        
        return new ApproachTrajectory(0, driveY, rotate, isAligned, isAtTarget);
    }
}
