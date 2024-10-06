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
public class drive extends LinearOpMode{
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor linearSlide1 = null;
    private DcMotor linearSlide2 = null;
    private Servo claw = null;
    private Servo extension = null;
    private Servo twist = null;
    private double twistVertMin = 0.0;
    private double twistVertMax = 0.5;
    private IMU imu = null;
    private double robotAngle;

    IMU.Parameters myIMUparameters;
    YawPitchRollAngles robotOrientation;

    // Extension right and left bumpers
    // Claw is B and A
    // Twist is right and left dpad
    // Linear slide is up and down dpad
    // Linear slide adjustments left y stick
    // Extension adustment is right x stick


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lfMtr");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lbMtr");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rfMtr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rbMtr");
        // Set directions of motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set 0 pwr behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize the linear slides
        linearSlide1 = hardwareMap.get(DcMotor.class, "ls1");
        linearSlide2 = hardwareMap.get(DcMotor.class, "ls2");

        // reset encoder counts kept by motors.
        linearSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set direction of linear slides
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize servos
        claw = hardwareMap.get(Servo.class, "claw");
        extension = hardwareMap.get(Servo.class, "extension");
        twist = hardwareMap.get(Servo.class, "twist");

        claw.setPosition(1);
        extension.setPosition(1);
        twist.setPosition(twistVertMin);

        // IMU
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu = hardwareMap.get(IMU.class, "imu");

        // Init localizer
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get Pose
            Pose2d myPose = drive.pose;

            // Get IMU data
            robotOrientation = imu.getRobotYawPitchRollAngles();

            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            robotAngle = myPose.heading.real; // Change to right one

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial_target   = -gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double lateral_target     =  -gamepad1.left_stick_y;
            double axial_real = axial_target*Math.sin(robotAngle) + lateral_target*Math.cos(robotAngle);
            double lateral_real = -axial_target*Math.cos(robotAngle) + lateral_target*Math.sin(robotAngle);
            double yaw     =  gamepad1.right_stick_x;

            double right_trigger = 1+gamepad1.right_trigger;
            double left_trigger = 1-gamepad1.left_trigger;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = ((axial_real + lateral_real + yaw)/2) * right_trigger * left_trigger;
            double rightFrontPower = ((axial_real - lateral_real - yaw)/2) * right_trigger * left_trigger;
            double leftBackPower   = ((axial_real - lateral_real + yaw)/2) * right_trigger * left_trigger;
            double rightBackPower  = ((axial_real + lateral_real - yaw)/2) * right_trigger * left_trigger;

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
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Linear slides
            if (gamepad2.dpad_down){
                // set motors to run forward for 5000 encoder counts.
                linearSlide1.setTargetPosition(3000);
                linearSlide2.setTargetPosition(3000);
                // set motors to run to target encoder position and stop with brakes on.
                linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set motors to run to target encoder position and stop with brakes on.
                linearSlide1.setPower(0.5);
                linearSlide2.setPower(0.5);
            }
            else if (gamepad2.dpad_up){
                linearSlide1.setTargetPosition(0);
                linearSlide2.setTargetPosition(0);
                linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlide1.setPower(0.5);
                linearSlide2.setPower(0.5);
            }
            // Adjustments
            double linearAdjustment = (gamepad2.left_stick_y*10);
            linearSlide1.setTargetPosition((int) (linearSlide1.getCurrentPosition()+linearAdjustment));
            linearSlide2.setTargetPosition((int) (linearSlide2.getCurrentPosition()+linearAdjustment));

            // Servos
            // Extension
            if (gamepad2.right_bumper){
                // set motors to run forward for 5000 encoder counts.
                extension.setPosition(1.0);
            }
            else if (gamepad2.left_bumper) {
                // set motors to run forward for 5000 encoder counts.
                extension.setPosition(0);
            }
            // Twist
            if (gamepad2.dpad_right){
                // set motors to run forward for 5000 encoder counts.
                twist.setPosition(twistVertMax);
            }
            else if (gamepad2.dpad_left) {
                // set motors to run forward for 5000 encoder counts.
                twist.setPosition(twistVertMin);
            }

            // Claw
            if (gamepad2.b){
                // set motors to run forward for 5000 encoder counts.
                claw.setPosition(1.0);
            }
            else if (gamepad2.x) {
                // set motors to run forward for 5000 encoder counts.
                claw.setPosition(0);
            }

            // Adjustments
            double extensionAdjustment = (gamepad2.right_stick_x/128);
            extension.setPosition(extension.getPosition()+extensionAdjustment);


            // Send calculated power to wheels, convert power to rpm
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position", "x: " + myPose.position.x + "y: " + myPose.position.y);
            telemetry.addData("Heading", "Angle: " + myPose.heading.real);
            telemetry.addData("Vert slides", "Position: " + linearSlide1.getCurrentPosition());
            telemetry.addData("Extension", "Position: " + extension.getPosition());
            telemetry.addData("Claw", "Position: " + claw.getPosition());
            telemetry.addData("Twist", "Position: " + twist.getPosition());
            telemetry.addData("Linear Slides", "LS1 Position: " + linearSlide1.getCurrentPosition() + "LS2 Position: " + linearSlide2.getCurrentPosition());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}