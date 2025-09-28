package org.firstinspires.ftc.teamcode.vision.utils;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

/**
 * Mathematical utilities for vision processing
 */
public class VisionMath {
    
    /**
     * Filter AprilTag detections by various criteria
     */
    public static class AprilTagFilter {
        
        /**
         * Filter tags by distance range
         */
        public static List<AprilTagDetection> filterByDistance(List<AprilTagDetection> detections,
                                                             double minDistance, double maxDistance) {
            if (detections == null) return new ArrayList<>();
            
            List<AprilTagDetection> filtered = new ArrayList<>();
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose != null) {
                    double distance = Math.sqrt(
                        Math.pow(detection.ftcPose.x, 2) + 
                        Math.pow(detection.ftcPose.z, 2)
                    );
                    if (distance >= minDistance && distance <= maxDistance) {
                        filtered.add(detection);
                    }
                }
            }
            return filtered;
        }
        
        /**
         * Filter tags by ID list
         */
        public static List<AprilTagDetection> filterByIds(List<AprilTagDetection> detections,
                                                        int... allowedIds) {
            if (detections == null) return new ArrayList<>();
            
            List<AprilTagDetection> filtered = new ArrayList<>();
            for (AprilTagDetection detection : detections) {
                for (int id : allowedIds) {
                    if (detection.id == id) {
                        filtered.add(detection);
                        break;
                    }
                }
            }
            return filtered;
        }
        
        /**
         * Sort tags by distance (closest first)
         */
        public static List<AprilTagDetection> sortByDistance(List<AprilTagDetection> detections) {
            if (detections == null) return new ArrayList<>();
            
            List<AprilTagDetection> sorted = new ArrayList<>(detections);
            Collections.sort(sorted, new Comparator<AprilTagDetection>() {
                @Override
                public int compare(AprilTagDetection a, AprilTagDetection b) {
                    if (a.ftcPose == null && b.ftcPose == null) return 0;
                    if (a.ftcPose == null) return 1;
                    if (b.ftcPose == null) return -1;
                    
                    double distA = Math.sqrt(Math.pow(a.ftcPose.x, 2) + Math.pow(a.ftcPose.z, 2));
                    double distB = Math.sqrt(Math.pow(b.ftcPose.x, 2) + Math.pow(b.ftcPose.z, 2));
                    
                    return Double.compare(distA, distB);
                }
            });
            return sorted;
        }
        
        /**
         * Get the most reliable tag (based on detection confidence and stability)
         */
        public static AprilTagDetection getMostReliable(List<AprilTagDetection> detections) {
            if (detections == null || detections.isEmpty()) return null;
            
            AprilTagDetection best = null;
            double bestScore = -1;
            
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose != null) {
                    // Score based on distance (closer is better) and decision margin
                    double distance = Math.sqrt(
                        Math.pow(detection.ftcPose.x, 2) + 
                        Math.pow(detection.ftcPose.z, 2)
                    );
                    
                    // Prefer tags that are:
                    // - Not too close (< 6 inches) or too far (> 60 inches)
                    // - Have good decision margin
                    double distanceScore = 1.0 / (1.0 + Math.abs(distance - 24.0)); // Prefer ~24 inches
                    double marginScore = detection.decisionMargin / 100.0; // Normalize decision margin
                    
                    double totalScore = (distanceScore + marginScore) / 2.0;
                    
                    if (totalScore > bestScore) {
                        bestScore = totalScore;
                        best = detection;
                    }
                }
            }
            
            return best;
        }
    }
    
    /**
     * Filter and analyze color blobs
     */
    public static class ColorBlobFilter {
        
        /**
         * Filter blobs by area range
         */
        public static List<ColorBlobLocatorProcessor.Blob> filterByArea(
                List<ColorBlobLocatorProcessor.Blob> blobs, double minArea, double maxArea) {
            if (blobs == null) return new ArrayList<>();
            
            List<ColorBlobLocatorProcessor.Blob> filtered = new ArrayList<>();
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                double area = blob.getContourArea();
                if (area >= minArea && area <= maxArea) {
                    filtered.add(blob);
                }
            }
            return filtered;
        }
        
        /**
         * Filter blobs by aspect ratio range
         */
        public static List<ColorBlobLocatorProcessor.Blob> filterByAspectRatio(
                List<ColorBlobLocatorProcessor.Blob> blobs, double minRatio, double maxRatio) {
            if (blobs == null) return new ArrayList<>();
            
            List<ColorBlobLocatorProcessor.Blob> filtered = new ArrayList<>();
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                double aspectRatio = blob.getAspectRatio();
                if (aspectRatio >= minRatio && aspectRatio <= maxRatio) {
                    filtered.add(blob);
                }
            }
            return filtered;
        }
        
        /**
         * Filter blobs by density (how filled the bounding box is)
         */
        public static List<ColorBlobLocatorProcessor.Blob> filterByDensity(
                List<ColorBlobLocatorProcessor.Blob> blobs, double minDensity) {
            if (blobs == null) return new ArrayList<>();
            
            List<ColorBlobLocatorProcessor.Blob> filtered = new ArrayList<>();
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                double density = blob.getDensity();
                if (density >= minDensity) {
                    filtered.add(blob);
                }
            }
            return filtered;
        }
        
        /**
         * Sort blobs by area (largest first)
         */
        public static List<ColorBlobLocatorProcessor.Blob> sortByArea(
                List<ColorBlobLocatorProcessor.Blob> blobs) {
            if (blobs == null) return new ArrayList<>();
            
            List<ColorBlobLocatorProcessor.Blob> sorted = new ArrayList<>(blobs);
            Collections.sort(sorted, new Comparator<ColorBlobLocatorProcessor.Blob>() {
                @Override
                public int compare(ColorBlobLocatorProcessor.Blob a, ColorBlobLocatorProcessor.Blob b) {
                    return Double.compare(b.getContourArea(), a.getContourArea());
                }
            });
            return sorted;
        }
        
        /**
         * Sort blobs by distance from image center
         */
        public static List<ColorBlobLocatorProcessor.Blob> sortByDistanceFromCenter(
                List<ColorBlobLocatorProcessor.Blob> blobs, int imageWidth, int imageHeight) {
            if (blobs == null) return new ArrayList<>();
            
            final double centerX = imageWidth / 2.0;
            final double centerY = imageHeight / 2.0;
            
            List<ColorBlobLocatorProcessor.Blob> sorted = new ArrayList<>(blobs);
            Collections.sort(sorted, new Comparator<ColorBlobLocatorProcessor.Blob>() {
                @Override
                public int compare(ColorBlobLocatorProcessor.Blob a, ColorBlobLocatorProcessor.Blob b) {
                    double distA = Math.sqrt(
                        Math.pow(a.getBoxFit().center.x - centerX, 2) + 
                        Math.pow(a.getBoxFit().center.y - centerY, 2)
                    );
                    double distB = Math.sqrt(
                        Math.pow(b.getBoxFit().center.x - centerX, 2) + 
                        Math.pow(b.getBoxFit().center.y - centerY, 2)
                    );
                    return Double.compare(distA, distB);
                }
            });
            return sorted;
        }
        
        /**
         * Get the best blob based on multiple criteria
         */
        public static ColorBlobLocatorProcessor.Blob getBestBlob(
                List<ColorBlobLocatorProcessor.Blob> blobs, 
                int imageWidth, int imageHeight,
                double preferredArea, double preferredAspectRatio) {
            if (blobs == null || blobs.isEmpty()) return null;
            
            ColorBlobLocatorProcessor.Blob best = null;
            double bestScore = -1;
            
            double centerX = imageWidth / 2.0;
            double centerY = imageHeight / 2.0;
            
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                // Score based on multiple factors
                double area = blob.getContourArea();
                double aspectRatio = blob.getAspectRatio();
                double density = blob.getDensity();
                
                // Distance from center of image
                double distFromCenter = Math.sqrt(
                    Math.pow(blob.getBoxFit().center.x - centerX, 2) + 
                    Math.pow(blob.getBoxFit().center.y - centerY, 2)
                );
                double maxDistance = Math.sqrt(centerX * centerX + centerY * centerY);
                
                // Normalize scores (0-1, higher is better)
                double areaScore = Math.exp(-Math.abs(area - preferredArea) / preferredArea);
                double aspectScore = Math.exp(-Math.abs(aspectRatio - preferredAspectRatio) / preferredAspectRatio);
                double densityScore = density; // Already 0-1
                double centerScore = 1.0 - (distFromCenter / maxDistance);
                
                // Weighted combination
                double totalScore = (areaScore * 0.3) + (aspectScore * 0.2) + (densityScore * 0.2) + (centerScore * 0.3);
                
                if (totalScore > bestScore) {
                    bestScore = totalScore;
                    best = blob;
                }
            }
            
            return best;
        }
    }
    
    /**
     * Coordinate transformation utilities
     */
    public static class CoordinateTransform {
        
        /**
         * Transform pixel coordinates to normalized coordinates (-1 to 1)
         */
        public static double[] pixelToNormalized(double pixelX, double pixelY, int imageWidth, int imageHeight) {
            double normalizedX = (2.0 * pixelX / imageWidth) - 1.0;
            double normalizedY = 1.0 - (2.0 * pixelY / imageHeight); // Flip Y axis
            return new double[]{normalizedX, normalizedY};
        }
        
        /**
         * Transform normalized coordinates to pixel coordinates
         */
        public static double[] normalizedToPixel(double normalizedX, double normalizedY, int imageWidth, int imageHeight) {
            double pixelX = (normalizedX + 1.0) * imageWidth / 2.0;
            double pixelY = (1.0 - normalizedY) * imageHeight / 2.0; // Flip Y axis
            return new double[]{pixelX, pixelY};
        }
        
        /**
         * Calculate angle from camera center to point
         */
        public static double[] pixelToAngle(double pixelX, double pixelY, int imageWidth, int imageHeight,
                                          double horizontalFOV, double verticalFOV) {
            double[] normalized = pixelToNormalized(pixelX, pixelY, imageWidth, imageHeight);
            double horizontalAngle = normalized[0] * horizontalFOV / 2.0;
            double verticalAngle = normalized[1] * verticalFOV / 2.0;
            return new double[]{horizontalAngle, verticalAngle};
        }
    }
    
    /**
     * Statistical analysis utilities
     */
    public static class Statistics {
        
        /**
         * Calculate moving average for stabilizing detections
         */
        public static class MovingAverage {
            private final double[] values;
            private int index = 0;
            private boolean filled = false;
            
            public MovingAverage(int size) {
                values = new double[size];
            }
            
            public void add(double value) {
                values[index] = value;
                index = (index + 1) % values.length;
                if (index == 0) filled = true;
            }
            
            public double getAverage() {
                if (!filled && index == 0) return 0; // No values yet
                
                double sum = 0;
                int count = filled ? values.length : index;
                for (int i = 0; i < count; i++) {
                    sum += values[i];
                }
                return sum / count;
            }
            
            public void reset() {
                index = 0;
                filled = false;
            }
        }
        
        /**
         * Calculate standard deviation of a list of values
         */
        public static double standardDeviation(List<Double> values) {
            if (values == null || values.size() < 2) return 0;
            
            double mean = 0;
            for (double value : values) {
                mean += value;
            }
            mean /= values.size();
            
            double variance = 0;
            for (double value : values) {
                variance += Math.pow(value - mean, 2);
            }
            variance /= values.size() - 1;
            
            return Math.sqrt(variance);
        }
        
        /**
         * Remove outliers from detection list based on distance from mean
         */
        public static List<Double> removeOutliers(List<Double> values, double threshold) {
            if (values == null || values.size() < 3) return values;
            
            double mean = 0;
            for (double value : values) {
                mean += value;
            }
            mean /= values.size();
            
            double stdDev = standardDeviation(values);
            
            List<Double> filtered = new ArrayList<>();
            for (double value : values) {
                if (Math.abs(value - mean) <= threshold * stdDev) {
                    filtered.add(value);
                }
            }
            
            return filtered;
        }
    }
    
    /**
     * Geometric calculations
     */
    public static class Geometry {
        
        /**
         * Calculate distance between two points
         */
        public static double distance(double x1, double y1, double x2, double y2) {
            return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        }
        
        /**
         * Calculate angle between two points (in radians)
         */
        public static double angle(double x1, double y1, double x2, double y2) {
            return Math.atan2(y2 - y1, x2 - x1);
        }
        
        /**
         * Normalize angle to [-π, π]
         */
        public static double normalizeAngle(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }
        
        /**
         * Calculate the intersection point of two lines (if it exists)
         */
        public static double[] lineIntersection(double x1, double y1, double x2, double y2,
                                              double x3, double y3, double x4, double y4) {
            double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (Math.abs(denom) < 1e-10) {
                return null; // Lines are parallel
            }
            
            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
            
            return new double[]{
                x1 + t * (x2 - x1),
                y1 + t * (y2 - y1)
            };
        }
    }
}
