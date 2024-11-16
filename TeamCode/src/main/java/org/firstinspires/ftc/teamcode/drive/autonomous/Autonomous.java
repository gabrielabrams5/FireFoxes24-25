package org.firstinspires.ftc.teamcode.drive.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TEST_AUTONOMOUS", group = "Autonomous")
public class Autonomous extends LinearOpMode {
    public static class Positions {
        public static final Pose2d BUCKET_BLUE = new Pose2d(49, -47, Math.toRadians(-45));
        public static final Pose2d BUCKET_RED = new Pose2d(-47.75, 45.75, Math.toRadians(135));

        public static final Pose2d SAMPLE_NEUTRAL_BLUE_FAR = new Pose2d(36.5, -27.5, Math.toRadians(0));
        public static final Pose2d SAMPLE_NEUTRAL_BLUE_MIDDLE = new Pose2d(46.5, -27.75, Math.toRadians(0));
        public static final Pose2d SAMPLE_NEUTRAL_BLUE_CLOSE = new Pose2d(56, -27.5, Math.toRadians(0));

        public static final Pose2d SAMPLE_RED_FAR = new Pose2d(35, -24, Math.toRadians(0));
        public static final Pose2d SAMPLE_RED_MIDDLE = new Pose2d(45, -24, Math.toRadians(0));
        public static final Pose2d SAMPLE_RED_CLOSE = new Pose2d(55, -24, Math.toRadians(0));

        public static final Pose2d SAMPLE_BLUE_FAR = new Pose2d(-35.5, 24, Math.toRadians(180));
        public static final Pose2d SAMPLE_BLUE_MIDDLE = new Pose2d(-45, 24, Math.toRadians(180));
        public static final Pose2d SAMPLE_BLUE_CLOSE = new Pose2d(-55, 24, Math.toRadians(180));

        public static final Pose2d SAMPLE_NEUTRAL_RED_FAR = new Pose2d(-36.5, 27.5, Math.toRadians(180));
        public static final Pose2d SAMPLE_NEUTRAL_RED_MIDDLE = new Pose2d(-46.5, 27.75, Math.toRadians(180));
        public static final Pose2d SAMPLE_NEUTRAL_RED_CLOSE = new Pose2d(-55.5, 27.5, Math.toRadians(180));
    }

    MecanumDrive.Params parameters = new MecanumDrive.Params();

    enum StartingPosition {
        BLUE_BUCKET(new Pose2d(35, -62, 0)),
        BLUE_DIVE(new Pose2d(0, 0, 0)),
        RED_BUCKET(new Pose2d(-35, 62, Math.toRadians(180))),
        RED_DIVE(new Pose2d(0, 0, 0));

        final Pose2d startPos;

        public Pose2d getStartPos() {
            return startPos;
        }

        StartingPosition(Pose2d startPos) {
            this.startPos = startPos;
        }
    }

    public class Robot {
        Lift lift;
        Extension extension;
        Twist twist;
        Claw claw;
        MecanumDrive drive;

        public Robot(Lift lift, Extension extension, Twist twist, Claw claw, MecanumDrive drive) {
            this.lift = lift;
            this.extension = extension;
            this.twist = twist;
            this.claw = claw;
            this.drive = drive;
        }

        public Action Init() {
            return new SequentialAction(
                    claw.clawInit(),
                    lift.liftInit(),
                    twist.twistInit(),
                    extension.extensionInit()
            );
        }

        public Action poseToBucket(TrajectoryActionBuilder poseToBucket) {
            return new SequentialAction(
                    new ParallelAction(
                            poseToBucket.build(),
                            lift.liftUp(),
                            twist.twistUp()
                    ),
                    new SleepAction(0.5),
                    extension.extensionBucket(),
                    new SleepAction(0.5),
                    claw.clawOpen(),
                    twist.twistUpUp(),
                    new SleepAction(0.75),
                    extension.extensionIn(),
                    claw.clawClose(),
                    new SleepAction(0.75)
            );
        }

        public Action bucketToSample(TrajectoryActionBuilder bucketToSample) {
            return new SequentialAction(
                    new ParallelAction(
                            bucketToSample.build(),
                            extension.extensionIn(),
                            twist.twistDown(),
                            lift.liftFloat(),
                            claw.clawOpen()
                    ),
                    GetSample()
            );
        }

        public Action GetSample() {
            return new SequentialAction(
                    extension.extensionOut(),
                    new SleepAction(0.5),
                    lift.liftBottom(),
                    new SleepAction(0.5),
                    claw.clawClose(),
                    new SleepAction(0.5),
                    lift.resetEncoders(),
                    new ParallelAction(
                            twist.twistUp(),
                            extension.extensionIn()
                    )
            );
        }

        public Action bucketToSubmersible(TrajectoryActionBuilder bucketToSubmersible) {
            return new ParallelAction(
                    bucketToSubmersible.build(),
                    extension.extensionIn(),
                    twist.twistDown(),
                    lift.liftFloat(),
                    claw.clawOpen()
            );
        }
    }

    public class Lift {
        private final DcMotorEx linearSlide1;
        private final DcMotorEx linearSlide2;

        int linearSlide1TargetPosition = parameters.LINEAR_SLIDE_START;
        int linearSlide2TargetPosition = parameters.LINEAR_SLIDE_START;

        public Lift(HardwareMap hardwareMap) {
            linearSlide1 = hardwareMap.get(DcMotorEx.class, "ls1");
            linearSlide2 = hardwareMap.get(DcMotorEx.class, "ls2");
            linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
            linearSlide2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftMove implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearSlide1.setPower(0);
                    linearSlide2.setPower(0);
                    initialized = true;
                }

                double linearSlide1Pos = linearSlide1.getCurrentPosition();
                double linearSlide2Pos = linearSlide2.getCurrentPosition();
                packet.put("Linear Slide 1 Position", linearSlide1Pos);
                packet.put("Linear Slide 1 Target Position", linearSlide1TargetPosition);
                packet.put("Linear Slide 2 Target Position", linearSlide2TargetPosition);
                packet.put("Linear Slide 2 Position", linearSlide2Pos);
                double linearSlide1Error = Math.abs(linearSlide1TargetPosition - linearSlide1Pos);
                double linearSlide2Error = Math.abs(linearSlide2TargetPosition - linearSlide2Pos);
                boolean isAbove1 = linearSlide1Pos > linearSlide1TargetPosition;
                boolean isAbove2 = linearSlide2Pos > linearSlide2TargetPosition;

                if (linearSlide1Error > 50) {
                    linearSlide1.setPower(isAbove1 ? -0.8 : 0.8);
//                    linearSlide2.setPower(isAbove1 ? -0.8 : 0.8);
                }
                else {
                    double slide1Power = 0.5 * linearSlide1Error / 50;
                    linearSlide1.setPower(isAbove1 ? -slide1Power*slide1Power : slide1Power*slide1Power);
//                    linearSlide2.setPower(isAbove1 ? -slide1Power*slide1Power : slide1Power*slide1Power);
                }

                if (linearSlide2Error > 50) {
                    linearSlide2.setPower(isAbove2 ? -0.8 : 0.8);
                } else {
                    double slide2Power = 0.5 * linearSlide2Error / 50;
                    linearSlide2.setPower(isAbove2 ? -slide2Power*slide2Power : slide2Power*slide2Power);
                }
                return true;
            }
        }

        public Action moveLift() {
            return new LiftMove();
        }

        public class LiftUp implements Action {
//            private boolean initialized = false;
//
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linearSlide1TargetPosition = parameters.LINEAR_SLIDE_MAX;
                linearSlide2TargetPosition = parameters.LINEAR_SLIDE_MAX;
                return false;
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class ResetEncoders implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }

        public Action resetEncoders() {
            return new ResetEncoders();
        }

        public class LiftFloat implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linearSlide1TargetPosition = parameters.LINEAR_SLIDE_FLOAT;
                linearSlide2TargetPosition = parameters.LINEAR_SLIDE_FLOAT;
                return false;
            }
        }

        public Action liftFloat() {
            return new LiftFloat();
        }

        public class LiftBottom implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linearSlide1TargetPosition = parameters.LINEAR_SLIDE_ROCK_BOTTOM;
                linearSlide2TargetPosition = parameters.LINEAR_SLIDE_ROCK_BOTTOM;
                return false;
            }
        }

        public Action liftBottom() {
            return new LiftBottom();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linearSlide1TargetPosition = parameters.LINEAR_SLIDE_MIN;
                linearSlide2TargetPosition = parameters.LINEAR_SLIDE_MIN;
                return false;
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

        public class LiftInit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearSlide1.setPower(
                        linearSlide1.getCurrentPosition() > parameters.LINEAR_SLIDE_START ? -0.5 : 0.5);
                    linearSlide2.setPower(
                        linearSlide2.getCurrentPosition() > parameters.LINEAR_SLIDE_START ? -0.5 : 0.5);
                    initialized = true;
                }

                double linearSlide1Position = linearSlide1.getCurrentPosition();
                double linearSlide2Position = linearSlide2.getCurrentPosition();
                boolean isLinearSlide1Initialized = false;
                boolean isLinearSlide2Initialized = false;
                packet.put("Linear Slide 1 Position", linearSlide1Position);
                packet.put("Linear Slide 2 Position", linearSlide2Position);
                packet.put("Linear Slide Target", parameters.LINEAR_SLIDE_START);
                if (Math.abs(linearSlide1Position - parameters.LINEAR_SLIDE_START) < 5 || Math.abs(linearSlide2Position - parameters.LINEAR_SLIDE_START) < 5) {
                    isLinearSlide1Initialized = true;
                    linearSlide1.setPower(0);
                    linearSlide2.setPower(0);
                }
                return !isLinearSlide1Initialized;
            }
        }

        public Action liftInit() {
            return new LiftInit();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class ClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(parameters.CLAW_CLOSE);
                return false;
            }
        }

        public Action clawClose() {
            return new ClawClose();
        }

        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(parameters.CLAW_OPEN);
                return false;
            }
        }

        public Action clawOpen() {
            return new ClawOpen();
        }

        public class ClawInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(parameters.AUTON_CLAW_INIT);
                return false;
            }
        }

        public Action clawInit() {
            return new ClawInit();
        }
    }

    public class Twist {
        private final DcMotorEx twist;
        private int targetPosition = parameters.TWIST_START;

        public Twist(HardwareMap hardwareMap) {
            twist = hardwareMap.get(DcMotorEx.class, "twist");
            twist.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            twist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            twist.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            twist.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            twist.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class TwistMove implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    double error = (twist.getCurrentPosition() - targetPosition);
                    error = error > 0 ? error : Math.abs(error*1.2);
                    twist.setVelocity(300*-(Math.cos(Math.PI * error/120)-1)/2);
                    initialized = true;
                }

                double pos = twist.getCurrentPosition();  // Assumes both slides at same pos
                packet.put("Twist Position", pos);
                packet.put("Twist Target Position", targetPosition);
                double error = (twist.getCurrentPosition() - targetPosition);
                if (error > 0){
                    twist.setVelocity(150*(Math.cos(Math.PI * error/120)-1)/2);
                } else{
                    error = 1.2*Math.abs(error);
                    twist.setVelocity(450*-(Math.cos(Math.PI * error/120)-1)/2);
                }
                return true;
            }
        }

        public Action moveTwist() {
            return new TwistMove();
        }

        public class TwistUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = parameters.TWIST_HIGH;
                return false;
//                if (!initialized) {
//                    double error = (twist.getCurrentPosition() - parameters.TWIST_HIGH);
//                    error = error > 0 ? error : Math.abs(error*1.2);
//                    twist.setPower(-(Math.cos(Math.PI * error/120)-1)/2);
//                    initialized = true;
//                }
//
//                double pos = twist.getCurrentPosition();  // Assumes both slides at same pos
//                packet.put("Twist Position", pos);
//                packet.put("Twist Target Position", parameters.TWIST_HIGH);
//                if (Math.abs(pos - parameters.TWIST_HIGH) > 5) {    // Keep raising lift if it hasn't reached max height yet
//                    double error = (twist.getCurrentPosition() - parameters.TWIST_HIGH);
//                    if (error > 0){
//                        twist.setPower((Math.cos(Math.PI * error/120)-1)/2);
//                    } else{
//                        error = 1.2*Math.abs(error);
//                        twist.setPower(-(Math.cos(Math.PI * error/120)-1)/2);
//                    }
//                    return true;
//                } else {
//                    // If lift is at desired position, stop raising
//                    twist.setPower(0.01);
//                    return false;
//                }
            }
        }

        public Action twistUp() {
            return new TwistUp();
        }

        public class TwistUpUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = parameters.TWIST_UPUP;
                return false;
            }
        }

        public Action twistUpUp() {
            return new TwistUpUp();
        }

        public class TwistDown implements Action {
            private boolean initialized = false;
            private int iterations = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = parameters.TWIST_LOW;
                while(iterations < 8) {
                    twist.setPower(-1);
                    iterations++;
                    return true;
                }
                return false;
//                if (!initialized) {
//                    double error = (twist.getCurrentPosition() - parameters.TWIST_LOW);
//                    twist.setPower((Math.cos(Math.PI * error/120)-1)/2);
//                    initialized = true;
//                }
//
//                double pos = twist.getCurrentPosition();
//                packet.put("Twist Position", pos);
//                packet.put("Twist Target Position", parameters.TWIST_LOW);
//                if (pos > parameters.TWIST_LOW) {    // Keep lowering lift if it hasn't reached max height yet
//                    double error = (twist.getCurrentPosition() - parameters.TWIST_LOW);
//                    twist.setPower((Math.cos(Math.PI * error/120)-1)/2);
//                    return true;
//                } else {
//                    // If lift is at desired position, stop lowering
//                    twist.setPower(0);
//                    return false;
//                }
            }
        }

        public Action twistDown() {
            return new TwistDown();
        }

        public class TwistInit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    twist.setPower(
                            twist.getCurrentPosition() > parameters.TWIST_START ? -0.2 : 0.2);
                    initialized = true;
                }

                double twistPosition = twist.getCurrentPosition();
                boolean isTwistInitialized = false;
                packet.put("Twist Position", twistPosition);
                packet.put("Twist Target Position", parameters.TWIST_START);
                if (Math.abs(twistPosition - parameters.TWIST_START) < 5) {
                    isTwistInitialized = true;
                    twist.setPower(0);
                }

                return !isTwistInitialized;
            }
        }

        public Action twistInit() {
            return new TwistInit();
        }
    }


    public class Extension {
        private Servo extension;

        public Extension(HardwareMap hardwareMap) {
            extension = hardwareMap.get(Servo.class, "extension");
        }

        public class ExtensionIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extension.setPosition(parameters.EXTENSION_IN);
                return false;
            }
        }

        public Action extensionIn() {
            return new ExtensionIn();
        }

        public class ExtensionBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extension.setPosition(parameters.EXTENSION_BUCKET);
                return false;
            }
        }

        public Action extensionBucket() {
            return new ExtensionBucket();
        }

        public class ExtensionOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extension.setPosition(parameters.EXTENSION_MIDDLE);
                return false;
            }
        }

        public Action extensionOut() {
            return new ExtensionOut();
        }

        public class ExtensionInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extension.setPosition(parameters.EXTENSION_START);
                return false;
            }
        }

        public Action extensionInit() {
            return new ExtensionInit();
        }
    }

    @Override
    public void runOpMode() {
        StartingPosition startPos = StartingPosition.RED_BUCKET;

        Pose2d initialPose = startPos.getStartPos();
        Robot robot = new Robot(
                new Lift(hardwareMap), new Extension(hardwareMap), new Twist(hardwareMap),
                new Claw(hardwareMap), new MecanumDrive(hardwareMap, initialPose));

        // Trajectories to select from

        /*
        Naming scheme for TrajectoryActionBuilders that go from buckets to blocks:
        {color of team}BucketTo{distance of target block from wall}{color of target block}Block
        ex.
        blueBucketToMiddleNeutralBlock

        Naming scheme for TrajectoryActionBuilders that go from blocks to buckets:
        {color of team}{distance of current block from wall}{color of block}BlockToBucket
        ex.
        blueMiddleNeutralBlockToBucket
        */

        TrajectoryActionBuilder blueInitToBucket = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(315));
        TrajectoryActionBuilder blueBucketToFarNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_FAR, Math.toRadians(-50));
        TrajectoryActionBuilder blueFarNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_FAR)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToMiddleNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(-225))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE, Math.toRadians(-90));
        TrajectoryActionBuilder blueMiddleNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToCloseNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(-225))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_CLOSE, Math.toRadians(-60));
        TrajectoryActionBuilder blueCloseNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_CLOSE)
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToSubmersible = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(26, -15, Math.toRadians(0)), Math.toRadians(200));


        TrajectoryActionBuilder redInitToBucket = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.BUCKET_RED, Math.toRadians(135));
        TrajectoryActionBuilder redBucketToFarNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_RED_FAR, Math.toRadians(130));
        TrajectoryActionBuilder redFarNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_FAR)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_RED, Math.toRadians(225));
        TrajectoryActionBuilder redBucketToMiddleNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_RED_MIDDLE, Math.toRadians(90));
        TrajectoryActionBuilder redMiddleNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_MIDDLE)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_RED, Math.toRadians(225));
        TrajectoryActionBuilder redBucketToCloseNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_RED_CLOSE, Math.toRadians(120));
        TrajectoryActionBuilder redCloseNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_CLOSE)
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(Positions.BUCKET_RED, Math.toRadians(225));
        TrajectoryActionBuilder redBucketToSubmersible = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-26, 15, Math.toRadians(180)), Math.toRadians(20));


        TrajectoryActionBuilder blueBucketToFarBlueBlock = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_FAR, Math.toRadians(-80));
        TrajectoryActionBuilder blueFarBlueBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_FAR)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToMiddleBlueBlock = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_MIDDLE, Math.toRadians(-90));
        TrajectoryActionBuilder blueMiddleBlueBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToCloseBlueBlock = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_CLOSE, Math.toRadians(-60));
        TrajectoryActionBuilder blueCloseBlueBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_CLOSE)
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));
        // TODO: Correctly invert the rest of these trajectories

        TrajectoryActionBuilder redBucketToFarRedBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(Positions.SAMPLE_RED_FAR, Math.toRadians(100));
        TrajectoryActionBuilder redFarRedBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_FAR)
                .splineToLinearHeading(Positions.BUCKET_RED, 45);
        TrajectoryActionBuilder redBucketToMiddleRedBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.SAMPLE_RED_MIDDLE, 0);
        TrajectoryActionBuilder redMiddleRedBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_MIDDLE)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_RED, 45);
        TrajectoryActionBuilder redBucketToCloseRedBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.SAMPLE_RED_CLOSE, 0);
        TrajectoryActionBuilder redCloseRedBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_CLOSE)
                .splineToLinearHeading(Positions.BUCKET_RED, 45);

        // Initialization Actions
        Actions.runBlocking(robot.Init());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("X Position during Init", robot.drive.pose.position.x);
            telemetry.addData("Y Position during Init", robot.drive.pose.position.y);
            telemetry.addData("Heading during Init", robot.drive.pose.heading.real);

            telemetry.update();
        }

        Action actionToExecute;

        switch(startPos) {
            case BLUE_BUCKET:
                actionToExecute = new SequentialAction(
                        robot.poseToBucket(blueInitToBucket),
                        robot.bucketToSample(blueBucketToFarNeutralBlock),
                        robot.poseToBucket(blueFarNeutralBlockToBucket),
                        robot.bucketToSample(blueBucketToMiddleNeutralBlock),
                        robot.poseToBucket(blueMiddleNeutralBlockToBucket),
                        robot.bucketToSample(blueBucketToCloseNeutralBlock),
                        robot.poseToBucket(blueCloseNeutralBlockToBucket),
                        robot.bucketToSubmersible(blueBucketToSubmersible)
                );
                break;
            case BLUE_DIVE:
                actionToExecute = new SequentialAction(
                        robot.poseToBucket(blueInitToBucket),
                        robot.bucketToSample(blueBucketToFarBlueBlock),
                        robot.poseToBucket(blueFarBlueBlockToBucket),
                        robot.bucketToSample(blueBucketToMiddleBlueBlock),
                        robot.poseToBucket(blueMiddleBlueBlockToBucket),
                        robot.bucketToSample(blueBucketToCloseBlueBlock),
                        robot.poseToBucket(blueCloseBlueBlockToBucket)
                );
                break;
            case RED_BUCKET:
                actionToExecute = new SequentialAction(
                        robot.poseToBucket(redInitToBucket),
                        robot.bucketToSample(redBucketToFarNeutralBlock),
                        robot.poseToBucket(redFarNeutralBlockToBucket),
                        robot.bucketToSample(redBucketToMiddleNeutralBlock),
                        robot.poseToBucket(redMiddleNeutralBlockToBucket),
                        robot.bucketToSample(redBucketToCloseNeutralBlock),
                        robot.poseToBucket(redCloseNeutralBlockToBucket),
                        robot.bucketToSubmersible(redBucketToSubmersible)
                );
                break;
            case RED_DIVE:
                actionToExecute = new SequentialAction(
                        robot.poseToBucket(redInitToBucket),
                        robot.bucketToSample(redBucketToFarRedBlock),
                        robot.poseToBucket(redFarRedBlockToBucket),
                        robot.bucketToSample(redBucketToMiddleRedBlock),
                        robot.poseToBucket(redMiddleRedBlockToBucket),
                        robot.bucketToSample(redBucketToCloseRedBlock),
                        robot.poseToBucket(redCloseRedBlockToBucket)
                );
            default:
                actionToExecute = robot.drive.actionBuilder(new Pose2d(0, 0, 0)).build();
                break;
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        robot.twist.moveTwist(),
                    robot.lift.moveLift(),
                    actionToExecute
                )
        );
    }
}
