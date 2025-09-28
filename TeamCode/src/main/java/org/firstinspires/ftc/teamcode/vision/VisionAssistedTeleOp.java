package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.vision.utils.VisionNavigation;
import org.firstinspires.ftc.teamcode.vision.utils.VisionMath;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

@Config
@TeleOp(name = "Vision Assisted TeleOp", group = "Vision")
public class VisionAssistedTeleOp extends LinearOpMode {
    
    // Tunable parameters
    public static double DRIVE_SPEED_NORMAL = 0.8;
    public static double DRIVE_SPEED_PRECISE = 0.3;
    public static double TURN_SPEED_NORMAL = 0.6;
    public static double TURN_SPEED_PRECISE = 0.2;
    
    // Vision assistance parameters
    public static boolean ENABLE_APRILTAG_ASSIST = true;
    public static boolean ENABLE_COLOR_ASSIST = true;
    public static double AUTO_ALIGN_SPEED = 0.3;
    public static double ALIGNMENT_TOLERANCE = 3.0; // degrees
    public static double DISTANCE_TOLERANCE = 2.0; // inches
    public static double TARGET_DISTANCE = 12.0; // inches from AprilTag
    public static ColorRange ASSIST_COLOR = ColorRange.YELLOW;
    
    // Vision tracking
    public static int PREFERRED_APRILTAG_ID = -1; // -1 for any tag
    public static double BLOB_TARGET_AREA = 1500.0;
    
    private CameraSubsystem camera;
    private ElapsedTime runtime = new ElapsedTime();
    
    // Drive motors
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    
    // Vision assistance controllers
    private VisionNavigation.VisionPIDController aprilTagHeadingController;
    private VisionNavigation.VisionPIDController aprilTagDistanceController;
    private VisionNavigation.VisionPIDController colorAlignController;
    
    // Assistance modes
    private enum AssistMode {
        MANUAL,           // Pure manual control
        APRILTAG_ALIGN,   // Align with AprilTag
        APRILTAG_APPROACH,// Approach AprilTag
        COLOR_TRACK,      // Track color object
        COLOR_APPROACH    // Approach color object
    }
    
    private AssistMode currentMode = AssistMode.MANUAL;
    private AprilTagDetection assistTarget = null;
    private ColorBlobLocatorProcessor.Blob colorTarget = null;
    
    @Override
    public void runOpMode() {
        // Initialize telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize drive motors
        initializeDriveMotors();
        
        // Initialize camera
        camera = new CameraSubsystem(hardwareMap, telemetry);
        camera.setAprilTagEnabled(ENABLE_APRILTAG_ASSIST);
        camera.setColorDetectionEnabled(ENABLE_COLOR_ASSIST);
        camera.setTargetColor(ASSIST_COLOR);
        
        // Initialize PID controllers
        aprilTagHeadingController = new VisionNavigation.VisionPIDController(0.015, 0.0, 0.003);
        aprilTagDistanceController = new VisionNavigation.VisionPIDController(0.02, 0.0, 0.005);
        colorAlignController = new VisionNavigation.VisionPIDController(0.01, 0.0, 0.002);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Vision Assisted TeleOp");
        telemetry.addLine("• Normal driving with vision assistance");
        telemetry.addLine("• Hold triggers for auto-align features");
        telemetry.addLine("• Bumpers for precision mode");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            // Update camera detections
            updateVisionDetections();
            
            // Handle mode switching
            updateAssistMode();
            
            // Calculate drive powers based on current mode
            double[] drivePowers = calculateDrivePowers();
            
            // Apply drive powers
            setDrivePowers(drivePowers[0], drivePowers[1], drivePowers[2], drivePowers[3]);
            
            // Handle other controls
            handleCameraControls();
            handleVisionSettings();
            
            // Update telemetry
            updateTelemetry();
            
            telemetry.update();
        }
        
        // Cleanup
        camera.close();
    }
    
    private void initializeDriveMotors() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Set motor directions (adjust based on your robot)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void updateVisionDetections() {
        if (ENABLE_APRILTAG_ASSIST) {
            camera.updateAprilTagDetections();
        }
        if (ENABLE_COLOR_ASSIST) {
            camera.updateColorDetections();
        }
    }
    
    private void updateAssistMode() {
        // Priority: Manual > AprilTag > Color
        if (gamepad1.left_trigger > 0.5 && ENABLE_APRILTAG_ASSIST) {
            // AprilTag alignment mode
            List<AprilTagDetection> detections = camera.getAprilTagDetections();
            if (detections != null && !detections.isEmpty()) {
                if (PREFERRED_APRILTAG_ID != -1) {
                    assistTarget = camera.getTargetAprilTag(PREFERRED_APRILTAG_ID);
                } else {
                    assistTarget = camera.getClosestAprilTag();
                }
                
                if (assistTarget != null) {
                    currentMode = AssistMode.APRILTAG_ALIGN;
                }
            }
        } else if (gamepad1.right_trigger > 0.5 && ENABLE_APRILTAG_ASSIST) {
            // AprilTag approach mode
            List<AprilTagDetection> detections = camera.getAprilTagDetections();
            if (detections != null && !detections.isEmpty()) {
                if (PREFERRED_APRILTAG_ID != -1) {
                    assistTarget = camera.getTargetAprilTag(PREFERRED_APRILTAG_ID);
                } else {
                    assistTarget = camera.getClosestAprilTag();
                }
                
                if (assistTarget != null) {
                    currentMode = AssistMode.APRILTAG_APPROACH;
                }
            }
        } else if (gamepad1.left_bumper && ENABLE_COLOR_ASSIST) {
            // Color tracking mode
            colorTarget = camera.getLargestBlob();
            if (colorTarget != null && colorTarget.getContourArea() > 100) {
                currentMode = AssistMode.COLOR_TRACK;
            }
        } else if (gamepad1.right_bumper && ENABLE_COLOR_ASSIST) {
            // Color approach mode
            colorTarget = camera.getLargestBlob();
            if (colorTarget != null && colorTarget.getContourArea() > 100) {
                currentMode = AssistMode.COLOR_APPROACH;
            }
        } else {
            // Manual mode when no assistance is active
            currentMode = AssistMode.MANUAL;
            assistTarget = null;
            colorTarget = null;
        }
    }
    
    private double[] calculateDrivePowers() {
        // Base manual inputs
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;  // Left/right
        double turn = gamepad1.right_stick_x;   // Rotation
        
        // Apply deadband
        drive = Math.abs(drive) > 0.05 ? drive : 0;
        strafe = Math.abs(strafe) > 0.05 ? strafe : 0;
        turn = Math.abs(turn) > 0.05 ? turn : 0;
        
        // Determine speed multiplier
        boolean precisionMode = gamepad1.dpad_down;
        double speedMultiplier = precisionMode ? DRIVE_SPEED_PRECISE : DRIVE_SPEED_NORMAL;
        double turnMultiplier = precisionMode ? TURN_SPEED_PRECISE : TURN_SPEED_NORMAL;
        
        // Apply vision assistance based on current mode
        switch (currentMode) {
            case APRILTAG_ALIGN:
                if (assistTarget != null) {
                    double bearing = Math.toDegrees(camera.getHeadingToTarget(assistTarget));
                    double turnCorrection = aprilTagHeadingController.calculate(Math.toRadians(bearing));
                    
                    // Override turn command with vision correction, but allow manual drive/strafe
                    turn = Range.clip(turnCorrection, -AUTO_ALIGN_SPEED, AUTO_ALIGN_SPEED);
                    
                    // Reduce manual drive speed during auto-align
                    drive *= 0.5;
                    strafe *= 0.5;
                }
                break;
                
            case APRILTAG_APPROACH:
                if (assistTarget != null) {
                    double distance = camera.getDistanceToTarget(assistTarget);
                    double bearing = Math.toDegrees(camera.getHeadingToTarget(assistTarget));
                    
                    double turnCorrection = aprilTagHeadingController.calculate(Math.toRadians(bearing));
                    double driveCorrection = aprilTagDistanceController.calculate(distance - TARGET_DISTANCE);
                    
                    // Blend manual and automatic control
                    turn = Range.clip(turn * 0.3 + turnCorrection, -AUTO_ALIGN_SPEED, AUTO_ALIGN_SPEED);
                    drive = Range.clip(drive * 0.3 - driveCorrection, -AUTO_ALIGN_SPEED, AUTO_ALIGN_SPEED);
                    strafe *= 0.3; // Reduce strafe during approach
                }
                break;
                
            case COLOR_TRACK:
                if (colorTarget != null) {
                    double angleError = Math.toRadians(camera.getBlobAngleFromCenter(colorTarget));
                    double turnCorrection = colorAlignController.calculate(angleError);
                    
                    // Override turn with vision correction
                    turn = Range.clip(turnCorrection, -AUTO_ALIGN_SPEED * 0.7, AUTO_ALIGN_SPEED * 0.7);
                    
                    // Allow manual drive but reduce speed
                    drive *= 0.6;
                    strafe *= 0.6;
                }
                break;
                
            case COLOR_APPROACH:
                if (colorTarget != null) {
                    double angleError = Math.toRadians(camera.getBlobAngleFromCenter(colorTarget));
                    double areaError = BLOB_TARGET_AREA - colorTarget.getContourArea();
                    
                    double turnCorrection = colorAlignController.calculate(angleError);
                    double driveCorrection = areaError > 0 ? AUTO_ALIGN_SPEED * 0.5 : -AUTO_ALIGN_SPEED * 0.3;
                    
                    // Blend manual and automatic control
                    turn = Range.clip(turn * 0.2 + turnCorrection, -AUTO_ALIGN_SPEED, AUTO_ALIGN_SPEED);
                    drive = Range.clip(drive * 0.2 + driveCorrection, -AUTO_ALIGN_SPEED, AUTO_ALIGN_SPEED);
                    strafe *= 0.2;
                }
                break;
                
            case MANUAL:
            default:
                // Pure manual control - no vision assistance
                break;
        }
        
        // Apply speed limits
        drive *= speedMultiplier;
        strafe *= speedMultiplier;
        turn *= turnMultiplier;
        
        // Calculate mecanum drive motor powers
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;
        
        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                                  Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }
        
        return new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
    }
    
    private void setDrivePowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        this.leftFront.setPower(leftFront);
        this.rightFront.setPower(rightFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
    }
    
    private void handleCameraControls() {
        // Toggle vision assistance features
        if (gamepad1.a) {
            ENABLE_APRILTAG_ASSIST = !ENABLE_APRILTAG_ASSIST;
            camera.setAprilTagEnabled(ENABLE_APRILTAG_ASSIST);
        }
        
        if (gamepad1.b) {
            ENABLE_COLOR_ASSIST = !ENABLE_COLOR_ASSIST;
            camera.setColorDetectionEnabled(ENABLE_COLOR_ASSIST);
        }
        
        // Camera exposure controls
        if (gamepad1.dpad_up) {
            camera.setAutoExposure(false);
            camera.setCameraExposure(6, 250); // Bright conditions
        }
        
        if (gamepad1.dpad_left) {
            camera.setAutoExposure(true);
        }
    }
    
    private void handleVisionSettings() {
        // Change target color
        if (gamepad1.x) {
            ASSIST_COLOR = ColorRange.RED;
            camera.setTargetColor(ASSIST_COLOR);
        }
        if (gamepad1.y) {
            ASSIST_COLOR = ColorRange.BLUE;
            camera.setTargetColor(ASSIST_COLOR);
        }
        
        // Change preferred AprilTag ID
        if (gamepad1.start) {
            PREFERRED_APRILTAG_ID = PREFERRED_APRILTAG_ID == -1 ? 1 : PREFERRED_APRILTAG_ID + 1;
            if (PREFERRED_APRILTAG_ID > 10) PREFERRED_APRILTAG_ID = -1;
        }
    }
    
    private void updateTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Current Mode", currentMode.toString());
        telemetry.addLine();
        
        // Drive information
        telemetry.addData("Drive", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f",
            leftFront.getPower(), rightFront.getPower(),
            leftBack.getPower(), rightBack.getPower());
        
        // Vision assistance status
        telemetry.addData("Vision Assistance", "");
        telemetry.addData("  AprilTag Assist", ENABLE_APRILTAG_ASSIST ? "ON" : "OFF");
        telemetry.addData("  Color Assist", ENABLE_COLOR_ASSIST ? "ON" : "OFF");
        telemetry.addData("  Target Color", ASSIST_COLOR.toString());
        telemetry.addData("  Preferred Tag ID", PREFERRED_APRILTAG_ID == -1 ? "ANY" : PREFERRED_APRILTAG_ID);
        
        // Current targets
        if (assistTarget != null && currentMode.toString().contains("APRILTAG")) {
            telemetry.addLine();
            telemetry.addData("AprilTag Target", "ID %d", assistTarget.id);
            telemetry.addData("  Distance", "%.1f inches", camera.getDistanceToTarget(assistTarget));
            telemetry.addData("  Bearing", "%.1f degrees", Math.toDegrees(camera.getHeadingToTarget(assistTarget)));
            
            boolean aligned = Math.abs(Math.toDegrees(camera.getHeadingToTarget(assistTarget))) < ALIGNMENT_TOLERANCE;
            boolean atDistance = Math.abs(camera.getDistanceToTarget(assistTarget) - TARGET_DISTANCE) < DISTANCE_TOLERANCE;
            telemetry.addData("  Status", "Aligned: %s, At Distance: %s",
                aligned ? "✓" : "✗", atDistance ? "✓" : "✗");
        }
        
        if (colorTarget != null && currentMode.toString().contains("COLOR")) {
            telemetry.addLine();
            telemetry.addData("Color Target", "Area: %.0f", colorTarget.getContourArea());
            telemetry.addData("  Center", "(%.0f, %.0f)", 
                camera.getBlobCenterX(colorTarget), camera.getBlobCenterY(colorTarget));
            telemetry.addData("  Angle", "%.1f degrees", camera.getBlobAngleFromCenter(colorTarget));
            
            boolean aligned = Math.abs(camera.getBlobAngleFromCenter(colorTarget)) < ALIGNMENT_TOLERANCE;
            boolean atTarget = Math.abs(colorTarget.getContourArea() - BLOB_TARGET_AREA) < 300;
            telemetry.addData("  Status", "Aligned: %s, At Target: %s",
                aligned ? "✓" : "✗", atTarget ? "✓" : "✗");
        }
        
        // Camera telemetry
        camera.updateTelemetry();
        
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("• Left trigger: AprilTag align");
        telemetry.addLine("• Right trigger: AprilTag approach");
        telemetry.addLine("• Left bumper: Color track");
        telemetry.addLine("• Right bumper: Color approach");
        telemetry.addLine("• D-pad down: Precision mode");
        telemetry.addLine("• A: Toggle AprilTag assist");
        telemetry.addLine("• B: Toggle color assist");
        telemetry.addLine("• X/Y: Change target color");
        telemetry.addLine("• START: Change target tag ID");
    }
}
