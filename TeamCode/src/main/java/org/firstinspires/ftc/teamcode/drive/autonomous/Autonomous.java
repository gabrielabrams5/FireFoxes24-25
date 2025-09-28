package org.firstinspires.ftc.teamcode.drive.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Config
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE_AUTONOMOUS", group = "Autonomous")
public class Autonomous extends LinearOpMode {
    public static class Positions {
        //TODO: CREATE NEW POSITIONAL CONSTANTS
        public static final Pose2d BUCKET_BLUE = new Pose2d(48, -48, Math.toRadians(-45));
        public static final Pose2d BUCKET_RED = new Pose2d(-47, 49, Math.toRadians(135));

        public static final Pose2d SAMPLE_NEUTRAL_BLUE_FAR = new Pose2d(35.5, -27.25, Math.toRadians(0));
        public static final Pose2d SAMPLE_NEUTRAL_BLUE_MIDDLE = new Pose2d(44.5, -27.75, Math.toRadians(0));
        public static final Pose2d SAMPLE_NEUTRAL_BLUE_CLOSE = new Pose2d(55, -27, Math.toRadians(0));

        public static final Pose2d SAMPLE_RED_FAR = new Pose2d(38, 27.5, Math.toRadians(0));
        public static final Pose2d SAMPLE_RED_MIDDLE = new Pose2d(46.5, 27.75, Math.toRadians(0));
        public static final Pose2d SAMPLE_RED_CLOSE = new Pose2d(56.5, 27.75, Math.toRadians(0));
        public static final Pose2d SAMPLE_RED_CLIP = new Pose2d(50, 60, Math.toRadians(90));

        public static final Pose2d SAMPLE_BLUE_FAR = new Pose2d(-38, -27.5, Math.toRadians(180));
        public static final Pose2d SAMPLE_BLUE_MIDDLE = new Pose2d(-46.5, -27.75, Math.toRadians(180));
        public static final Pose2d SAMPLE_BLUE_CLOSE = new Pose2d(-56.5, -27.75, Math.toRadians(180));
        public static final Pose2d SAMPLE_BLUE_CLIP = new Pose2d(-50, -60, Math.toRadians(-90));
        public static final Pose2d SAMPLE_BLUE_CLIP_CLOSE = new Pose2d(-50, -48, Math.toRadians(-90));
        public static final Pose2d SAMPLE_BLUE_HANG = new Pose2d(0, -44, Math.toRadians(90));

        public static final Pose2d SAMPLE_NEUTRAL_RED_FAR = new Pose2d(-34, 25.5, Math.toRadians(180));
        public static final Pose2d SAMPLE_NEUTRAL_RED_MIDDLE = new Pose2d(-44, 27, Math.toRadians(180));
        public static final Pose2d SAMPLE_NEUTRAL_RED_CLOSE = new Pose2d(-56, 27, Math.toRadians(180));
    }

    MecanumDrive.Params parameters = new MecanumDrive.Params();

    enum StartingPosition {
        BLUE_BUCKET(new Pose2d(35, -62, 0)),
        BLUE_DIVE(new Pose2d(-12, -62, Math.toRadians(180))),
        RED_BUCKET(new Pose2d(-35, 62, Math.toRadians(180))),
        RED_DIVE(new Pose2d(35, 62, 0));

        final Pose2d startPos;

        public Pose2d getStartPos() {
            return startPos;
        }

        StartingPosition(Pose2d startPos) {
            this.startPos = startPos;
        }
    }

    public class Robot {
        Intake intake;
        Launch launch;
        Load load;
        MecanumDrive drive;

        public Robot(Intake intake, Launch launch, Load load, MecanumDrive drive) {
            this.intake = intake;
            this.launch = launch;
            this.load = load;
            this.drive = drive;
        }

        public Action Init() {
            return new SequentialAction(
                    intake.intakeInit(),
                    launch.launchInit(),
                    load.loadInit()
            );
        }

        public Action poseToBucket(TrajectoryActionBuilder poseToBucket) {
            return new SequentialAction(
                    new ParallelAction(
                            poseToBucket.build() // Movement + manipulators
                    ),
                    new SleepAction(0.5), // Sequential movements
                    new SleepAction(0.1)
            );
        }

        public Action bucketToSample(TrajectoryActionBuilder bucketToSample) {
            return new SequentialAction(
                    new SleepAction(0.1),
                    new ParallelAction(
                            bucketToSample.build()
                    ),
                    GetSample()
            );
        }

        public Action GetSample() {
            return new SequentialAction(
                    new SleepAction(0.5),
                    new SleepAction(0.5),
                    new SleepAction(0.5),
                    new ParallelAction(
                    )
            );
        }

        public Action GetSampleLow() {
            return new SequentialAction(
                    new SleepAction(0.5), // Sequential movements
                    new SleepAction(0.5),
                    new SleepAction(0.5)
            );
        }

        public Action bucketToSubmersible(TrajectoryActionBuilder bucketToSubmersible) {
            return new ParallelAction(
                    bucketToSubmersible.build()
            );
        }

        public Action poseToClip(TrajectoryActionBuilder poseToClip) {
            return new SequentialAction(
                    new ParallelAction(
                            poseToClip.build()
                    ),
                    new SleepAction(0.5)
            );
        }

        public Action clipToSample(TrajectoryActionBuilder clipToSample) {
            return new SequentialAction(
                    new ParallelAction(
                            clipToSample.build()
                    )
            );
        }

        public Action clipInchFoward(TrajectoryActionBuilder clipInchFoward){
            return new SequentialAction(
                    new ParallelAction(
                            clipInchFoward.build()
                    )
            );
        }

        public Action sampleToClip(TrajectoryActionBuilder sampleToClip) {
            return new SequentialAction(
                    new ParallelAction(
                            sampleToClip.build()
                    )
            );
        }

        public Action clipToHang(TrajectoryActionBuilder clipToHang) {
            return new SequentialAction(
                    new ParallelAction(
                            clipToHang.build()
                    ),
                    new SleepAction(0.75)
            );
        }

        public Action hangToSample(TrajectoryActionBuilder hangToSample) {
            return new SequentialAction(
                    new ParallelAction(
                            hangToSample.build()
                    ),
                    GetSample()
            );
        }
    }

    public class Intake {
        private final DcMotorEx intake;

        int intakeDirection = parameters.INTAKE_DIRECTION_START;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class IntakeMove implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0);
                    initialized = true;
                }

                if (intakeDirection == 1) {
                    intake.setPower(parameters.INTAKE_SPEED_IN);
                } else if (intakeDirection == -1) {
                    intake.setPower(parameters.INTAKE_SPEED_OUT);
                } else {
                    intake.setPower(0f);
                }

                return true;
            }
        }

        public Action moveIntake() {
            return new IntakeMove();
        }

        public class IntakeIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = 1;
                return false;
            }
        }

        public Action intakeIn() {
            return new IntakeIn();
        }

        public class IntakeOut implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = -1;
                return false;
            }
        }

        public Action intakeOut() {
            return new IntakeOut();
        }

        public class IntakeOff implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = 0;
                return false;
            }
        }

        public Action intakeOff() {
            return new IntakeOff();
        }

        public class IntakeInit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public Action intakeInit() {
            return new IntakeInit();
        }
    }

    public class Load {
        private Servo load;

        public Load(HardwareMap hardwareMap) {
            load = hardwareMap.get(Servo.class, "load");
        }

        public class LoadLoad implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_LOAD);
                return false;
            }
        }

        public Action loadLoad() {
            return new LoadLoad();
        }

        public class LoadReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_RESET);
                return false;
            }
        }

        public Action loadReset() {
            return new LoadReset();
        }

        public class LoadInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_INIT);
                return false;
            }
        }

        public Action loadInit() {
            return new LoadInit();
        }
    }

    public class Launch {
        private final DcMotorEx launch;

        boolean isLaunchActive = false;

        public Launch(HardwareMap hardwareMap) {
            launch = hardwareMap.get(DcMotorEx.class, "launch");
            launch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            launch.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LaunchMove implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                if (isLaunchActive) {
                    launch.setPower(parameters.LAUNCH_POWER);
                }

                return true;
            }
        }

        public Action moveLaunch() {
            return new LaunchMove();
        }

        public class LaunchOn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                isLaunchActive = true;
               return false;
            }
        }

        public Action launchOn() {
            return new LaunchOn();
        }

        public class LaunchOff implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                isLaunchActive = false;
                return false;
            }
        }

        public class LaunchInit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public Action launchInit() {
            return new LaunchInit();
        }
    }

    @Override
    public void runOpMode() {
        StartingPosition startPos = StartingPosition.RED_DIVE;

        Pose2d initialPose = startPos.getStartPos();
        Robot robot = new Robot(
                new Intake(hardwareMap), new Launch(hardwareMap), new Load(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose));

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
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(26, -10, Math.toRadians(0)), Math.toRadians(200));


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


        TrajectoryActionBuilder blueInitToClip = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_CLIP, Math.toRadians(225));
        TrajectoryActionBuilder blueClipToFarBlueBlock = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_CLIP)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_FAR, Math.toRadians(180));
        TrajectoryActionBuilder farBlueBlockToBlueClip = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_FAR)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_CLIP, Math.toRadians(90));
        TrajectoryActionBuilder clipToMediumBlueBlock = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_CLIP)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_MIDDLE, Math.toRadians(180));
        TrajectoryActionBuilder mediumBlueBlockToBlueClip = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_MIDDLE)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_CLIP, Math.toRadians(90));
        TrajectoryActionBuilder clipToCloseBlueBlock = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_CLIP)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_CLOSE, Math.toRadians(180));
        TrajectoryActionBuilder closeBlueBlockToBlueClip = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_CLOSE)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_CLIP, Math.toRadians(90));

        TrajectoryActionBuilder redInitToClip = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(25))
                .splineToLinearHeading(Positions.SAMPLE_RED_CLIP, Math.toRadians(270));
        TrajectoryActionBuilder redClipToFarRedBlock = robot.drive.actionBuilder(Positions.SAMPLE_RED_CLIP)
                .setTangent(Math.toRadians(235))
                .splineToLinearHeading(Positions.SAMPLE_RED_FAR, Math.toRadians(0));
        TrajectoryActionBuilder farRedBlockToRedClip = robot.drive.actionBuilder(Positions.SAMPLE_RED_FAR)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(Positions.SAMPLE_RED_CLIP, Math.toRadians(270));
        TrajectoryActionBuilder clipToMediumRedBlock = robot.drive.actionBuilder(Positions.SAMPLE_RED_CLIP)
                .setTangent(Math.toRadians(235))
                .splineToLinearHeading(Positions.SAMPLE_RED_MIDDLE, Math.toRadians(0));
        TrajectoryActionBuilder mediumRedBlockToRedClip = robot.drive.actionBuilder(Positions.SAMPLE_RED_MIDDLE)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(Positions.SAMPLE_RED_CLIP, Math.toRadians(270));
        TrajectoryActionBuilder clipToCloseRedBlock = robot.drive.actionBuilder(Positions.SAMPLE_RED_CLIP)
                .setTangent(Math.toRadians(235))
                .splineToLinearHeading(Positions.SAMPLE_RED_CLOSE, Math.toRadians(0));
        TrajectoryActionBuilder closeRedBlockToRedClip = robot.drive.actionBuilder(Positions.SAMPLE_RED_CLOSE)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(Positions.SAMPLE_RED_CLIP, Math.toRadians(-90));

        // Legacy code (possible stuff for clipping specimens:
        MinVelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(5),
                new AngularVelConstraint(1)
        ));
        TrajectoryActionBuilder blueClipInchFoward = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_CLIP)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_CLIP_CLOSE, Math.toRadians(90), velConstraint);
        TrajectoryActionBuilder blueClipToHang = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_CLIP)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_HANG, Math.toRadians(90));
        TrajectoryActionBuilder blueHangToMediumBlueBlock = robot.drive.actionBuilder(Positions.SAMPLE_BLUE_HANG)
                .setTangent(Math.toRadians(250))
                .splineToLinearHeading(Positions.SAMPLE_BLUE_MIDDLE, Math.toRadians(160));

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
                        robot.poseToClip(blueInitToClip),
                        robot.clipToSample(blueClipToFarBlueBlock),
                        robot.sampleToClip(farBlueBlockToBlueClip),
                        robot.clipToSample(clipToMediumBlueBlock),
                        robot.sampleToClip(mediumBlueBlockToBlueClip),
                        robot.clipToSample(clipToCloseBlueBlock),
                        robot.sampleToClip(closeBlueBlockToBlueClip)
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
                        robot.poseToClip(redInitToClip),
                        robot.clipToSample(redClipToFarRedBlock),
                        robot.sampleToClip(farRedBlockToRedClip),
                        robot.clipToSample(clipToMediumRedBlock),
                        robot.sampleToClip(mediumRedBlockToRedClip),
                        robot.clipToSample(clipToCloseRedBlock),
                        robot.sampleToClip(closeRedBlockToRedClip)
                );
                break;
            default:
                actionToExecute = robot.drive.actionBuilder(new Pose2d(0, 0, 0)).build();
                break;
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Run Pathing
        Actions.runBlocking(
                new ParallelAction(
                        robot.intake.moveIntake(),
                        robot.launch.moveLaunch(),
                        actionToExecute
                )
        );
    }
}
