package org.firstinspires.ftc.teamcode.drive.autonomous;
import android.media.audiofx.BassBoost;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
        public static final Pose2d BUCKET_BLUE = new Pose2d(56, 56, Math.toRadians(45));
        public static final Pose2d BUCKET_RED = new Pose2d(-56, -56, Math.toRadians(225));

        public static final Pose2d SAMPLE_NEUTRAL_BLUE_FAR = new Pose2d(35, 26, Math.toRadians(0));
        public static final Pose2d SAMPLE_NEUTRAL_BLUE_MIDDLE = new Pose2d(45, 26, Math.toRadians(0));
        public static final Pose2d SAMPLE_NEUTRAL_BLUE_CLOSE = new Pose2d(55, 26, Math.toRadians(0));

        public static final Pose2d SAMPLE_RED_FAR = new Pose2d(35, -26, Math.toRadians(0));
        public static final Pose2d SAMPLE_RED_MIDDLE = new Pose2d(45, -26, Math.toRadians(0));
        public static final Pose2d SAMPLE_RED_CLOSE = new Pose2d(55, -26, Math.toRadians(0));

        public static final Pose2d SAMPLE_BLUE_FAR = new Pose2d(-35, 26, Math.toRadians(180));
        public static final Pose2d SAMPLE_BLUE_MIDDLE = new Pose2d(-45, 26, Math.toRadians(180));
        public static final Pose2d SAMPLE_BLUE_CLOSE = new Pose2d(-55, 26, Math.toRadians(180));

        public static final Pose2d SAMPLE_NEUTRAL_RED_FAR = new Pose2d(-35, -26, Math.toRadians(180));
        public static final Pose2d SAMPLE_NEUTRAL_RED_MIDDLE = new Pose2d(-45, -26, Math.toRadians(180));
        public static final Pose2d SAMPLE_NEUTRAL_RED_CLOSE = new Pose2d(-55, -26, Math.toRadians(180));
    }

    MecanumDrive.Params parameters = new MecanumDrive.Params();

    enum StartingPosition {
        BLUE_BUCKET(new Pose2d(35, 62, 0)),
        BLUE_DIVE(new Pose2d(0, 0, 0)),
        RED_BUCKET(new Pose2d(-35, -62, 180)),
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
                    twist.twistInit(),
                    extension.extensionInit(),
                    lift.liftInit()
            );
        }

        public Action poseToBucket(TrajectoryActionBuilder poseToBucket) {
            return new SequentialAction(
                    new ParallelAction(
                            poseToBucket.build(),
                            lift.liftUp()
//                            twist.twistUp()
                    ),
                    claw.clawOpen()
            );
        }

        public Action bucketToSample(TrajectoryActionBuilder bucketToSample) {
            return new SequentialAction(
                    new ParallelAction(
                            bucketToSample.build(),
                            extension.extensionIn(),
//                            twist.twistDown(),
                            lift.liftDown(),
                            claw.clawOpen()
                    ),
                    GetSample()
            );
        }

        public Action GetSample() {
            return new SequentialAction(
                    extension.extensionOut(),
                    claw.clawClose(),
                    new ParallelAction(
//                            twist.twistUp()
                            extension.extensionIn()
                    )
            );
        }

        public Action HighBucket() {
            return new SequentialAction(
                    lift.liftUp(),
                    extension.extensionOut(),
                    claw.clawOpen(),
                    extension.extensionIn()
            );
        }
    }

    public class Lift {
        private final DcMotorEx linearSlide1;
        private final DcMotorEx linearSlide2;

        public Lift(HardwareMap hardwareMap) {
            linearSlide1 = hardwareMap.get(DcMotorEx.class, "linearSlide1");
            linearSlide2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");
            linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
            linearSlide2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearSlide1.setPower(0.8);
                    linearSlide2.setPower(0.8);
                    initialized = true;
                }

                double pos = linearSlide1.getCurrentPosition();  // Assumes both slides at same pos
                packet.put("Linear Slide Positions", pos);
                if (pos < parameters.LINEAR_SLIDE_MAX) {    // Keep raising lift if it hasn't reached max height yet
                    return true;
                } else {
                    // If lift is at desired position, stop raising
                    linearSlide1.setPower(0);
                    linearSlide2.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearSlide1.setPower(-0.8);
                    linearSlide2.setPower(-0.8);
                    initialized = true;
                }

                double pos = linearSlide1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > parameters.LINEAR_SLIDE_MIN) {    // Keep lowering lift if it hasn't reached max height yet
                    return true;
                } else {
                    // If lift is at desired position, stop raising
                    linearSlide1.setPower(0);
                    return false;
                }
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
                if (Math.abs(linearSlide1Position - parameters.LINEAR_SLIDE_START) < 5) {
                    isLinearSlide1Initialized = true;
                    linearSlide1.setPower(0);
                }
                if (Math.abs(linearSlide2Position - parameters.LINEAR_SLIDE_START) < 5) {
                    isLinearSlide2Initialized = true;
                    linearSlide2.setPower(0);
                }
                return !isLinearSlide2Initialized || !isLinearSlide1Initialized;
            }
        }

        public Action liftInit() {
            return new LiftInit();
        }
    }

    public class Claw {
        Servo claw;

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
                claw.setPosition(parameters.CLAW_START);
                return false;
            }
        }

        public Action clawInit() {
            return new ClawInit();
        }
    }

    public class Twist {
        DcMotor twist;

        public Twist(HardwareMap hardwareMap) {
            twist = hardwareMap.get(DcMotor.class, "twist");
        }

        public class TwistUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                twist.setPosition(parameters.TWIST_HIGH);
                return false;
            }
        }

        public Action twistUp() {
            return new TwistUp();
        }

        public class TwistDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                twist.setPosition(parameters.TWIST_LOW);
                return false;
            }
        }

        public Action twistDown() {
            return new TwistDown();
        }

        public class TwistInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                twist.setPosition(parameters.TWIST_START);
                return false;
            }
        }

        public Action twistInit() {
            return new TwistInit();
        }
    }

    public class Extension {
        Servo Extension;

        public Extension(HardwareMap hardwareMap) {
            Extension = hardwareMap.get(Servo.class, "extension");
        }

        public class ExtensionOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Extension.setPosition(parameters.EXTENSION_OUT);
                return false;
            }
        }

        public Action extensionOut() {
            return new ExtensionOut();
        }

        public class ExtensionIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Extension.setPosition(parameters.EXTENSION_IN);
                return false;
            }
        }

        public Action extensionIn() {
            return new ExtensionIn();
        }

        public class ExtensionInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Extension.setPosition(parameters.EXTENSION_START);
                return false;
            }
        }

        public Action extensionInit() {
            return new ExtensionInit();
        }
    }

    @Override
    public void runOpMode() {
        StartingPosition startPos = StartingPosition.BLUE_BUCKET;

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
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_FAR, Math.toRadians(-80));
        TrajectoryActionBuilder blueFarNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_FAR)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToMiddleNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE, Math.toRadians(-90));
        TrajectoryActionBuilder blueMiddleNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToCloseNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_CLOSE, Math.toRadians(-60));
        TrajectoryActionBuilder blueCloseNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_BLUE_CLOSE)
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45));

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

        TrajectoryActionBuilder blueBucketToSubmersible = robot.drive.actionBuilder(Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(26, 10, Math.toRadians(180)), Math.toRadians(270));

        TrajectoryActionBuilder redInitToBucket = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.BUCKET_RED, Math.toRadians(135));
        TrajectoryActionBuilder redBucketToFarNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_RED_FAR, Math.toRadians(100));
        TrajectoryActionBuilder redFarNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_FAR)
                .splineToLinearHeading(Positions.BUCKET_RED, 45);
        TrajectoryActionBuilder redBucketToMiddleNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_RED_MIDDLE, 0);
        TrajectoryActionBuilder redMiddleNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_MIDDLE)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_RED, 45);
        TrajectoryActionBuilder redBucketToCloseNeutralBlock = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_RED_CLOSE, 0);
        TrajectoryActionBuilder redCloseNeutralBlockToBucket = robot.drive.actionBuilder(Positions.SAMPLE_NEUTRAL_RED_CLOSE)
                .splineToLinearHeading(Positions.BUCKET_RED, 45);

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
        TrajectoryActionBuilder redBucketToSubmersible = robot.drive.actionBuilder(Positions.BUCKET_RED)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(26, 10, Math.toRadians(180)), Math.toRadians(270));

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
                        robot.poseToBucket(blueCloseNeutralBlockToBucket)
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
                        robot.poseToBucket(redCloseNeutralBlockToBucket)
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
                actionToExecute
        );
    }
}
