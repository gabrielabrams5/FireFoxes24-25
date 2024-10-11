package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
     * Extension:
     *   OUT    - Right Bumper
     *   IN     - Left Bumper
     *   ADJUST - Right Stick X
     * Claw:
     *   OPEN   - X Button
     *   CLOSE  - Y Button
     * Twist:
     *   HIGH   - Right dPad
     *   LOW    - Left dPad
     * Linear Slide:
     *   UP     - Up dPad
     *   DOWN   - Down dPad
     *   ADJUST - Left Stick Y
     */

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################

        // Initialize drive motor variables
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfMtr");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbMtr");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfMtr");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbMtr");
        // Set drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // Set drive motor zero power behaviors
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize linear slide motors
        DcMotor linearSlide1 = hardwareMap.get(DcMotor.class, "ls1");
        DcMotor linearSlide2 = hardwareMap.get(DcMotor.class, "ls2");
        // Reset linear slide encoders
        linearSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set linear slide motor directions
        linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize servos
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo extension = hardwareMap.get(Servo.class, "extension");
        Servo twist = hardwareMap.get(Servo.class, "twist");

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        // Initialize localizer
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        // Initialize mechanical position constants
        final double CLAW_START = 0.75;
        final double CLAW_OPEN = 0.75;
        final double CLAW_CLOSE = 0.45;
        final double EXTENSION_START = 0;
        final double EXTENSION_OUT = 1.0;
        final double EXTENSION_IN = 0;
        final double TWIST_START = 0.75;
        final double TWIST_HIGH = 0.0;
        final double TWIST_LOW = 0.5;
        final int LINEAR_SLIDE_MIN = 200;
        final int LINEAR_SLIDE_MAX = 2500;
        // Initialize mechanical position variables
        int linearSlide1Target = 0;
        int linearSlide2Target = 0;
        // Initialize robot position variables
        double robotAngle;
        YawPitchRollAngles robotOrientation;

        // Set servos to start positions
        claw.setPosition(CLAW_START);
        extension.setPosition(EXTENSION_START);
        twist.setPosition(TWIST_START);

        // Robot is ready to start! Display message to screen
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get Pose
            Pose2d myPose = drive.pose;

            // Get IMU data
            robotOrientation = imu.getRobotYawPitchRollAngles();

            // What are these used for?
            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll = robotOrientation.getRoll(AngleUnit.DEGREES);

            robotAngle = myPose.heading.real; // CHANGE TO RIGHT ONE!!!

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial_target = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral_target = gamepad1.left_stick_x;
            double axial_real = lateral_target * Math.cos(robotAngle) + axial_target * Math.sin(robotAngle);
            double lateral_real = lateral_target * -Math.sin(robotAngle) + axial_target * Math.cos(robotAngle);
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

            // Linear Slides
            if (gamepad2.dpad_up) {
                linearSlide1Target = LINEAR_SLIDE_MAX;
                linearSlide2Target = LINEAR_SLIDE_MAX;
            } else if (gamepad2.dpad_down) {
                linearSlide1Target = LINEAR_SLIDE_MIN;
                linearSlide2Target = LINEAR_SLIDE_MIN;
            }

            // Linear Slide Adjustments
            int linearAdjustment = (int) (gamepad2.left_stick_y * 20);
            linearSlide1Target -= linearAdjustment;
            linearSlide2Target -= linearAdjustment;

            // Give Linear Slides target positions and power
            linearSlide1.setTargetPosition(linearSlide1Target);
            linearSlide2.setTargetPosition(linearSlide2Target);
            linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide1.setPower(0.5);
            linearSlide2.setPower(0.5);

            // Extension Servo
            if (gamepad2.right_bumper) {
                extension.setPosition(EXTENSION_OUT);
            } else if (gamepad2.left_bumper) {
                extension.setPosition(EXTENSION_IN);
            }
            extension.setPosition(extension.getPosition() + (gamepad2.right_stick_x / 128));

            // Twist Servo
            if (gamepad2.dpad_right) {
                twist.setPosition(TWIST_LOW);
            } else if (gamepad2.dpad_left) {
                twist.setPosition(TWIST_HIGH);
            }
            // Claw Servo
            if (gamepad2.b) {
                claw.setPosition(CLAW_OPEN);
            } else if (gamepad2.x) {
                claw.setPosition(CLAW_CLOSE);
            }

            // Send calculated power to wheels, convert power to rpm
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Position", "x: " + myPose.position.x + "y: " + myPose.position.y);
            telemetry.addData("Heading", "Angle: " + myPose.heading.real);
            telemetry.addData("Extension: ", extension.getPosition());
            telemetry.addData("Claw: ", claw.getPosition());
            telemetry.addData("Twist: ", twist.getPosition());
            telemetry.addData("Linear Slides Real", "LS1 Position: " + linearSlide1.getCurrentPosition() + "LS2 Position: " + linearSlide2.getCurrentPosition());
            telemetry.addData("Linear Slides Target", "LS1 Target: " + linearSlide1Target + "LS2 Target: " + linearSlide2Target);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}