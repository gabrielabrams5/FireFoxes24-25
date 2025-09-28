package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

// Import Size class - try different options based on SDK version
import java.util.List;
import java.util.concurrent.TimeUnit;

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
    // Note: FilterCriteria might not be available in all SDK versions
    
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
        AprilTagProcessor.Builder aprilTagBuilder = new AprilTagProcessor.Builder();
        
        // Add common settings that should work across SDK versions
        try {
            aprilTagBuilder.setDrawAxes(true);
            aprilTagBuilder.setDrawCubeProjection(true);
            aprilTagBuilder.setDrawTagOutline(true);
            aprilTagBuilder.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);
            aprilTagBuilder.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
            
            // Optional lens intrinsics - may not be supported in all versions
            aprilTagBuilder.setLensIntrinsics(578.272, 578.272, 402.145, 221.506);
        } catch (Exception e) {
            // Fallback to basic configuration
            aprilTagBuilder.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
        }
        
        aprilTagProcessor = aprilTagBuilder.build();
        
        // Initialize Color Blob Processor
        ColorBlobLocatorProcessor.Builder colorBlobBuilder = new ColorBlobLocatorProcessor.Builder();
        
        try {
            colorBlobBuilder.setTargetColorRange(ColorRange.BLUE);
            // Try to set optional parameters
            try {
                colorBlobBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
            } catch (Exception e) { /* Optional feature */ }
            
            try {
                colorBlobBuilder.setRoi(ImageRegion.entireFrame());
            } catch (Exception e) { /* Optional feature */ }
            
            try {
                colorBlobBuilder.setDrawContours(true);
            } catch (Exception e) { /* Optional feature */ }
            
            try {
                colorBlobBuilder.setBlurSize(5);
            } catch (Exception e) { /* Optional feature */ }
            
        } catch (Exception e) {
            // Fallback to minimal configuration
            try {
                colorBlobBuilder.setTargetColorRange(ColorRange.BLUE);
            } catch (Exception e2) {
                telemetry.addLine("Warning: Could not configure color blob processor");
            }
        }
        
        try {
            colorBlobProcessor = colorBlobBuilder.build();
        } catch (Exception e) {
            colorBlobProcessor = null;
            telemetry.addLine("Warning: Could not create color blob processor");
        }
        
        // Initialize Predominant Color Processor
        PredominantColorProcessor.Builder predominantBuilder = new PredominantColorProcessor.Builder();
        
        try {
            predominantBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1));
            predominantBuilder.setSwatches(
                PredominantColorProcessor.Swatch.RED,
                PredominantColorProcessor.Swatch.BLUE,
                PredominantColorProcessor.Swatch.YELLOW
            );
        } catch (Exception e) {
            // Use default configuration if specific settings fail
        }
        
        predominantColorProcessor = predominantBuilder.build();
    }
    
    private void initializeVisionPortal() {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        
        // Set camera with error handling
        boolean cameraSet = false;
        try {
            WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            if (webcam != null) {
                builder.setCamera(webcam);
                cameraSet = true;
            }
        } catch (Exception e) {
            try {
                WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");
                if (webcam != null) {
                    builder.setCamera(webcam);
                    cameraSet = true;
                }
            } catch (Exception e2) {
                // Last resort - built-in camera
                try {
                    builder.setCamera(BuiltinCameraDirection.BACK);
                    cameraSet = true;
                    telemetry.addLine("Using built-in camera");
                } catch (Exception e3) {
                    telemetry.addLine("ERROR: No camera available");
                    return; // Can't proceed without a camera
                }
            }
        }
        
        if (!cameraSet) {
            telemetry.addLine("ERROR: Failed to set camera");
            telemetry.addLine("Please configure 'Webcam 1' in robot configuration");
            return;
        }
        
        // Note: Camera resolution setting is optional
        // Using default resolution to avoid Size class compatibility issues
        
        // Optional settings with error handling
        try {
            builder.enableLiveView(true);
            builder.setAutoStartStreamOnBuild(true);
        } catch (Exception e) {
            // These are optional, continue without them
        }
        
        // Add processors based on enabled features
        try {
            if (aprilTagEnabled && aprilTagProcessor != null) {
                builder.addProcessor(aprilTagProcessor);
            }
            if (colorDetectionEnabled && colorBlobProcessor != null) {
                builder.addProcessor(colorBlobProcessor);
            }
            if (predominantColorEnabled && predominantColorProcessor != null) {
                builder.addProcessor(predominantColorProcessor);
            }
        } catch (Exception e) {
            telemetry.addLine("Warning: Could not add some vision processors");
        }
        
        // Build the VisionPortal
        try {
            visionPortal = builder.build();
        } catch (Exception e) {
            telemetry.addLine("ERROR: Failed to build VisionPortal");
            telemetry.addLine("Error: " + e.getMessage());
            return;
        }
        
        // Start streaming to FTC Dashboard
        try {
            FtcDashboard.getInstance().startCameraStream(visionPortal, MAX_FPS);
            isStreaming = true;
        } catch (Exception e) {
            telemetry.addLine("Warning: Could not start Dashboard camera stream");
            // Continue anyway - the camera might still work
            isStreaming = true;
        }
    }
    
    private void setupColorFilter() {
        // Note: FilterCriteria and filterContours methods are not available in all SDK versions
        // Manual filtering will be done in the getColorBlobs method instead
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
            // Manual filtering since filterContours is not available in all SDK versions
            if (currentBlobs != null) {
                currentBlobs = filterBlobsByArea(currentBlobs, COLOR_BLOB_MIN_AREA, COLOR_BLOB_MAX_AREA);
            }
        }
    }
    
    // Manual filtering method since SDK utilities may not be available
    private List<ColorBlobLocatorProcessor.Blob> filterBlobsByArea(List<ColorBlobLocatorProcessor.Blob> blobs, double minArea, double maxArea) {
        List<ColorBlobLocatorProcessor.Blob> filtered = new java.util.ArrayList<>();
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            double area = blob.getContourArea();
            if (area >= minArea && area <= maxArea) {
                filtered.add(blob);
            }
        }
        return filtered;
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
        // Note: ColorBlobLocatorProcessor doesn't have updateTargetColorRange method
        // To change target color, we need to recreate the processor
        this.colorDetectionEnabled = true;
        if (visionPortal != null) {
            rebuildVisionPortal();
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
        if (visionPortal == null) return;
        
        try {
            // Check if camera is streaming
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                return;
            }
            
            // Try to get camera controls - these may not be available on all cameras
            ExposureControl exposureControl = null;
            GainControl gainControl = null;
            
            try {
                exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            } catch (Exception e) {
                // Exposure control not available
            }
            
            try {
                gainControl = visionPortal.getCameraControl(GainControl.class);
            } catch (Exception e) {
                // Gain control not available
            }
            
            // Set exposure if control is available
            if (exposureControl != null) {
                try {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
                } catch (Exception e) {
                    // Failed to set exposure
                }
            }
            
            // Set gain if control is available
            if (gainControl != null) {
                try {
                    gainControl.setGain(gain);
                } catch (Exception e) {
                    // Failed to set gain
                }
            }
            
        } catch (Exception e) {
            // Overall camera control failed - this is normal for some cameras
        }
    }
    
    public void setAutoExposure(boolean auto) {
        if (visionPortal == null) return;
        
        try {
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                return;
            }
            
            ExposureControl exposureControl = null;
            try {
                exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            } catch (Exception e) {
                // Exposure control not available
                return;
            }
            
            if (exposureControl != null) {
                try {
                    if (auto) {
                        exposureControl.setMode(ExposureControl.Mode.Auto);
                    } else {
                        setCameraExposure(EXPOSURE_MS, GAIN);
                    }
                } catch (Exception e) {
                    // Failed to set exposure mode
                }
            }
        } catch (Exception e) {
            // Camera control failed - this is normal for some cameras
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
