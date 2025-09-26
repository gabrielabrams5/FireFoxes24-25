package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

@Config
public class CameraSubsystem {
    // Tunable parameters via FTC Dashboard
    public static double APRIL_TAG_SIZE = 0.166; // meters (6.5 inches)
    public static int DESIRED_TAG_ID = -1; // -1 for any tag
    
    // Color detection parameters
    public static double COLOR_BLOB_MIN_AREA = 50.0;
    public static double COLOR_BLOB_MAX_AREA = 10000.0;
    public static double COLOR_EROSION = 10.0;
    public static double COLOR_DILATION = 10.0;
    
    // Camera parameters
    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT = 480;
    public static int MAX_FPS = 30;
    public static boolean AUTO_EXPOSURE = true;
    public static int EXPOSURE_MS = 6;
    public static int GAIN = 250;
    
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private ColorBlobLocatorProcessor colorBlobProcessor;
    private PredominantColorProcessor predominantColorProcessor;
    
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    
    // Detection results
    private List<AprilTagDetection> currentAprilTagDetections;
    private List<ColorBlobLocatorProcessor.Blob> currentBlobs;
    private ColorBlobLocatorProcessor.Util.FilterCriteria colorFilterCriteria;
    
    // Camera state
    private boolean isStreaming = false;
    private boolean aprilTagEnabled = true;
    private boolean colorDetectionEnabled = false;
    private boolean predominantColorEnabled = false;
    
    public CameraSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        
        initializeProcessors();
        initializeVisionPortal();
        setupColorFilter();
    }
    
    private void initializeProcessors() {
        // Initialize AprilTag Processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) // Default calibration
                .build();
        
        // Initialize Color Blob Processor
        colorBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // Default to blue
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
        
        // Initialize Predominant Color Processor
        predominantColorProcessor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                    PredominantColorProcessor.Swatch.RED,
                    PredominantColorProcessor.Swatch.BLUE,
                    PredominantColorProcessor.Swatch.YELLOW
                )
                .build();
    }
    
    private void initializeVisionPortal() {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        
        // Set camera
        if (hardwareMap.get(WebcamName.class, "Webcam 1") != null) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Fallback to built-in camera if webcam not found
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        }
        
        // Set camera resolution
        builder.setCameraResolution(new android.graphics.Size(CAMERA_WIDTH, CAMERA_HEIGHT));
        
        // Enable/disable live view (saves bandwidth)
        builder.enableLiveView(true);
        
        // Set auto-start camera streaming
        builder.setAutoStartStreamOnBuild(true);
        
        // Add processors based on enabled features
        if (aprilTagEnabled) {
            builder.addProcessor(aprilTagProcessor);
        }
        if (colorDetectionEnabled) {
            builder.addProcessor(colorBlobProcessor);
        }
        if (predominantColorEnabled) {
            builder.addProcessor(predominantColorProcessor);
        }
        
        visionPortal = builder.build();
        
        // Start streaming to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, MAX_FPS);
        isStreaming = true;
    }
    
    private void setupColorFilter() {
        colorFilterCriteria = new ColorBlobLocatorProcessor.Util.FilterCriteria()
                .setMinArea(COLOR_BLOB_MIN_AREA)
                .setMaxArea(COLOR_BLOB_MAX_AREA)
                .setMinCircularity(0.3)
                .setMaxCircularity(1.0)
                .setMinInertiaRatio(0.15);
    }
    
    // === AprilTag Methods ===
    
    public void updateAprilTagDetections() {
        if (aprilTagProcessor != null) {
            currentAprilTagDetections = aprilTagProcessor.getDetections();
        }
    }
    
    public List<AprilTagDetection> getAprilTagDetections() {
        return currentAprilTagDetections;
    }
    
    public AprilTagDetection getTargetAprilTag(int tagId) {
        if (currentAprilTagDetections != null) {
            for (AprilTagDetection detection : currentAprilTagDetections) {
                if (detection.id == tagId) {
                    return detection;
                }
            }
        }
        return null;
    }
    
    public AprilTagDetection getClosestAprilTag() {
        if (currentAprilTagDetections == null || currentAprilTagDetections.isEmpty()) {
            return null;
        }
        
        AprilTagDetection closest = null;
        double minDistance = Double.MAX_VALUE;
        
        for (AprilTagDetection detection : currentAprilTagDetections) {
            if (detection.ftcPose != null) {
                double distance = Math.sqrt(
                    Math.pow(detection.ftcPose.x, 2) + 
                    Math.pow(detection.ftcPose.y, 2) + 
                    Math.pow(detection.ftcPose.z, 2)
                );
                if (distance < minDistance) {
                    minDistance = distance;
                    closest = detection;
                }
            }
        }
        
        return closest;
    }
    
    // === Color Detection Methods ===
    
    public void updateColorDetections() {
        if (colorBlobProcessor != null) {
            currentBlobs = colorBlobProcessor.getBlobs();
            // Apply filtering
            if (currentBlobs != null && colorFilterCriteria != null) {
                currentBlobs = ColorBlobLocatorProcessor.Util.filterContours(currentBlobs, colorFilterCriteria);
            }
        }
    }
    
    public List<ColorBlobLocatorProcessor.Blob> getColorBlobs() {
        return currentBlobs;
    }
    
    public ColorBlobLocatorProcessor.Blob getLargestBlob() {
        if (currentBlobs == null || currentBlobs.isEmpty()) {
            return null;
        }
        
        ColorBlobLocatorProcessor.Blob largest = null;
        double maxArea = 0;
        
        for (ColorBlobLocatorProcessor.Blob blob : currentBlobs) {
            if (blob.getContourArea() > maxArea) {
                maxArea = blob.getContourArea();
                largest = blob;
            }
        }
        
        return largest;
    }
    
    public ColorBlobLocatorProcessor.Blob getClosestBlob() {
        if (currentBlobs == null || currentBlobs.isEmpty()) {
            return null;
        }
        
        ColorBlobLocatorProcessor.Blob closest = null;
        double minDistance = Double.MAX_VALUE;
        double centerX = CAMERA_WIDTH / 2.0;
        double centerY = CAMERA_HEIGHT / 2.0;
        
        for (ColorBlobLocatorProcessor.Blob blob : currentBlobs) {
            double blobCenterX = blob.getBoxFit().center.x;
            double blobCenterY = blob.getBoxFit().center.y;
            double distance = Math.sqrt(
                Math.pow(blobCenterX - centerX, 2) + 
                Math.pow(blobCenterY - centerY, 2)
            );
            
            if (distance < minDistance) {
                minDistance = distance;
                closest = blob;
            }
        }
        
        return closest;
    }
    
    // === Configuration Methods ===
    
    public void setTargetColor(ColorRange color) {
        if (colorBlobProcessor != null) {
            colorBlobProcessor.updateTargetColorRange(color);
        }
    }
    
    public void setAprilTagEnabled(boolean enabled) {
        this.aprilTagEnabled = enabled;
        // Note: Processor changes require recreating VisionPortal
        if (visionPortal != null) {
            rebuildVisionPortal();
        }
    }
    
    public void setColorDetectionEnabled(boolean enabled) {
        this.colorDetectionEnabled = enabled;
        if (visionPortal != null) {
            rebuildVisionPortal();
        }
    }
    
    public void setPredominantColorEnabled(boolean enabled) {
        this.predominantColorEnabled = enabled;
        if (visionPortal != null) {
            rebuildVisionPortal();
        }
    }
    
    // === Camera Control Methods ===
    
    public void startStreaming() {
        if (visionPortal != null && !isStreaming) {
            visionPortal.resumeStreaming();
            isStreaming = true;
        }
    }
    
    public void stopStreaming() {
        if (visionPortal != null && isStreaming) {
            visionPortal.stopStreaming();
            isStreaming = false;
        }
    }
    
    public void setCameraExposure(int exposureMs, int gain) {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // Camera controls may not be available on all cameras
            try {
                visionPortal.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.class)
                    .setMode(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.Mode.Manual);
                visionPortal.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.class)
                    .setExposure(exposureMs, java.util.concurrent.TimeUnit.MILLISECONDS);
                visionPortal.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl.class)
                    .setGain(gain);
            } catch (Exception e) {
                telemetry.addLine("Camera exposure control not available");
            }
        }
    }
    
    public void setAutoExposure(boolean auto) {
        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            try {
                if (auto) {
                    visionPortal.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.class)
                        .setMode(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.Mode.Auto);
                } else {
                    setCameraExposure(EXPOSURE_MS, GAIN);
                }
            } catch (Exception e) {
                telemetry.addLine("Camera exposure control not available");
            }
        }
    }
    
    // === Utility Methods ===
    
    public double getHeadingToTarget(AprilTagDetection target) {
        if (target == null || target.ftcPose == null) {
            return 0;
        }
        
        return Math.atan2(target.ftcPose.x, target.ftcPose.z);
    }
    
    public double getDistanceToTarget(AprilTagDetection target) {
        if (target == null || target.ftcPose == null) {
            return Double.MAX_VALUE;
        }
        
        return Math.sqrt(
            Math.pow(target.ftcPose.x, 2) + 
            Math.pow(target.ftcPose.z, 2)
        );
    }
    
    public double getBlobCenterX(ColorBlobLocatorProcessor.Blob blob) {
        if (blob == null) return CAMERA_WIDTH / 2.0;
        return blob.getBoxFit().center.x;
    }
    
    public double getBlobCenterY(ColorBlobLocatorProcessor.Blob blob) {
        if (blob == null) return CAMERA_HEIGHT / 2.0;
        return blob.getBoxFit().center.y;
    }
    
    public double getBlobAngleFromCenter(ColorBlobLocatorProcessor.Blob blob) {
        if (blob == null) return 0;
        
        double blobCenterX = getBlobCenterX(blob);
        double imageCenterX = CAMERA_WIDTH / 2.0;
        double offsetX = blobCenterX - imageCenterX;
        
        // Estimate field of view (typically around 60-90 degrees for webcams)
        double horizontalFOV = 78.0; // degrees
        double pixelsPerDegree = CAMERA_WIDTH / horizontalFOV;
        
        return offsetX / pixelsPerDegree;
    }
    
    // === Telemetry Methods ===
    
    public void updateTelemetry() {
        telemetry.addData("Camera Status", visionPortal.getCameraState());
        telemetry.addData("Streaming", isStreaming);
        
        if (aprilTagEnabled && currentAprilTagDetections != null) {
            telemetry.addData("AprilTags Detected", currentAprilTagDetections.size());
            
            for (AprilTagDetection detection : currentAprilTagDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("Tag %d (%s): Range %.1f\", Bearing %.0f°, Yaw %.0f°",
                        detection.id, 
                        detection.metadata.name,
                        getDistanceToTarget(detection),
                        Math.toDegrees(getHeadingToTarget(detection)),
                        detection.ftcPose != null ? detection.ftcPose.yaw : 0));
                }
            }
        }
        
        if (colorDetectionEnabled && currentBlobs != null) {
            telemetry.addData("Color Blobs Detected", currentBlobs.size());
            
            ColorBlobLocatorProcessor.Blob largest = getLargestBlob();
            if (largest != null) {
                telemetry.addData("Largest Blob Area", largest.getContourArea());
                telemetry.addData("Largest Blob Center", "%.0f, %.0f", 
                    getBlobCenterX(largest), getBlobCenterY(largest));
                telemetry.addData("Largest Blob Angle", "%.1f°", getBlobAngleFromCenter(largest));
            }
        }
    }
    
    // === Cleanup ===
    
    private void rebuildVisionPortal() {
        if (visionPortal != null) {
            visionPortal.close();
        }
        initializeVisionPortal();
    }
    
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    // === Static Helper Methods ===
    
    public static class VisionUtils {
        /**
         * Convert pixel coordinates to field coordinates based on AprilTag detection
         */
        public static double[] pixelToFieldCoordinates(AprilTagDetection tag, double pixelX, double pixelY) {
            if (tag == null || tag.ftcPose == null) {
                return new double[]{0, 0};
            }
            
            // This is a simplified conversion - actual implementation would require
            // camera calibration and perspective transformation
            double fieldX = tag.ftcPose.x + (pixelX - CAMERA_WIDTH/2.0) * 0.01;
            double fieldY = tag.ftcPose.z + (pixelY - CAMERA_HEIGHT/2.0) * 0.01;
            
            return new double[]{fieldX, fieldY};
        }
        
        /**
         * Calculate the angle error for aligning robot with a target
         */
        public static double calculateAlignmentError(ColorBlobLocatorProcessor.Blob target) {
            if (target == null) return 0;
            
            double targetCenterX = target.getBoxFit().center.x;
            double imageCenterX = CAMERA_WIDTH / 2.0;
            double offsetPixels = targetCenterX - imageCenterX;
            
            // Convert pixel offset to angle (assumes ~78° horizontal FOV)
            double degreesPerPixel = 78.0 / CAMERA_WIDTH;
            return offsetPixels * degreesPerPixel;
        }
    }
}
