package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

@TeleOp(name="Basic Drive", group="Linear OpMode")
public class drive extends LinearOpMode {
    /*
     * Intake:
     *    IN (TOGGLE ON/OFF)  - A
     *    OUT (TOGGLE ON/OFF) - B
     *    FLIP                - X
     *    STOP                - Y
     * Launcher:
     *    ON/OFF (TOGGLE)     - Right Bumper
     *    ON/OFF (HOLD)       - Right Trigger
     * Load:
     *    LOAD                - Dpad Right
     *    RESET               - Dpad Left
     * Macros:
     *    AIM + SHOOT         - Dpad Up
     */

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        // Initialize and configure drive motor variables
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfMtr");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbMtr");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfMtr");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbMtr");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize and configure intake motor
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize and configure launch motor
        DcMotorEx launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setDirection(DcMotorSimple.Direction.FORWARD);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize servos
        Servo load = hardwareMap.get(Servo.class, "load");

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize localizer and robot position variables. Get position constants
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        MecanumDrive.Params parameters = new MecanumDrive.Params();
        double robotAngle = 0;
        YawPitchRollAngles robotOrientation;

        // Initialize control parameters
        int intakeDirection = parameters.INTAKE_DIRECTION_START;
        boolean isLaunchActive = parameters.LAUNCH_START;
        boolean previousRightTrigger = false;

        // Robot is ready to start! Display message to screen
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get Pose
            drive.updatePoseEstimate();
            Pose2d myPose = drive.pose;

            // Get IMU data
            robotOrientation = imu.getRobotYawPitchRollAngles();

            // What are these used for?
            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll = robotOrientation.getRoll(AngleUnit.DEGREES);

            if (myPose != null) robotAngle = myPose.heading.toDouble(); // TODO: Change to right one

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial_target = gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double lateral_target = -gamepad1.left_stick_y;

            double theta = gamepad1.left_bumper ? -robotAngle : 0;
            double cosine = Math.cos(theta);
            double sine = Math.sin(theta);

            double targetx = axial_target * cosine - lateral_target * sine;
            double targety = axial_target * sine + lateral_target * cosine;

            double lateral_real = targetx;
            double axial_real = targety;

            // commented out perspective driving controls
//            double axial_real = lateral_target * Math.cos(robotAngle) + axial_target * Math.sin(robotAngle);
//            double lateral_real = lateral_target * -Math.sin(robotAngle) + axial_target * Math.cos(robotAngle);
            double yaw = gamepad1.right_stick_x;

            double right_trigger = 1 + gamepad1.right_trigger;
            double left_trigger = 1 - gamepad1.left_trigger;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = ((axial_real + lateral_real + yaw) / 2) * right_trigger * left_trigger;
            double rightFrontPower = ((axial_real - lateral_real - yaw) / 2) * right_trigger * left_trigger;
            double leftBackPower = ((axial_real - lateral_real + yaw) / 2) * right_trigger * left_trigger;
            double rightBackPower = ((axial_real + lateral_real - yaw) / 2) * right_trigger * left_trigger;

            /*leftFrontPower  = leftFrontPower>=0 ? leftFrontPower+right_trigger : leftFrontPower-right_trigger;
            rightFrontPower  = rightFrontPower>=0 ? rightFrontPower+right_trigger : rightFrontPower-right_trigger;
            leftBackPower  = leftBackPower>=0 ? leftBackPower+right_trigger : leftBackPower-right_trigger;
            rightBackPower  = rightBackPower>=0 ? rightBackPower+right_trigger : rightBackPower-right_trigger;

            leftFrontPower  = leftFrontPower>=0 ? leftFrontPower+left_trigger : leftFrontPower-left_trigger;
            rightFrontPower  = rightFrontPower>=0 ? rightFrontPower+left_trigger : rightFrontPower-left_trigger;
            leftBackPower  = leftBackPower>=0 ? leftBackPower+left_trigger : leftBackPower-left_trigger;
            rightBackPower  = rightBackPower>=0 ? rightBackPower+left_trigger : rightBackPower-left_trigger;*/

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                                  Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Intake
            if (gamepad1.a) {
                intakeDirection = intakeDirection == 1 ? 0 : 1;
            } else if (gamepad1.b) {
                intakeDirection = intakeDirection == -1 ? 0 : -1;
            } else if (gamepad1.x) {
                intakeDirection = intakeDirection == 1 ? -1 : 1;
            } else if (gamepad1.y) {
                intakeDirection = 0;
            }

            // Power Intake
            if (intakeDirection == 1) {
                intake.setPower(parameters.INTAKE_SPEED_IN);
            } else if (intakeDirection == -1) {
                intake.setPower(parameters.INTAKE_SPEED_OUT);
            } else {
                intake.setPower(0f);
            }

            // Launch
            if (gamepad1.right_trigger > 0.5f && !previousRightTrigger) {
                previousRightTrigger = true;
                isLaunchActive = !isLaunchActive;
            } else if (gamepad1.right_trigger < 0.5f) {
                previousRightTrigger = false;
            } else {
                isLaunchActive = gamepad1.right_bumper;
            }

            // Power Launch
            if (isLaunchActive) {
                launch.setPower(parameters.LAUNCH_POWER);
            }

            // Loader Servo
            if (gamepad1.dpad_right) {
                load.setPosition(parameters.LOAD_LOAD);
            } else if (gamepad1.dpad_left) {
                load.setPosition(parameters.LOAD_RESET);
            }

            // Aim + Launch Macro TODO: WRITE MACROS
            if (gamepad1.dpad_up) {
                //win
            }

            // Power Wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            if (myPose != null) {
                telemetry.addData("Position", "x: " + myPose.position.x + "y: " + myPose.position.y);
                telemetry.addData("Heading", "Angle: " + myPose.heading.toDouble());
            }
            telemetry.addData("Intake", "Direction: " + intakeDirection);
            telemetry.addData("Launch", "Active: " + isLaunchActive);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}