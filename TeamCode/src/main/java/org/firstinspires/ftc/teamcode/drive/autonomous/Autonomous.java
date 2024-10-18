package org.firstinspires.ftc.teamcode.drive.autonomous;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
        public final Vector2d BUCKET_BLUE = new Vector2d(56, 56);
        public final Vector2d BUCKET_RED = new Vector2d(-56, -56);

        public final Vector2d SAMPLE_NEUTRAL_BLUE_FAR = new Vector2d(40, 26);
        public final Vector2d SAMPLE_NEUTRAL_BLUE_MIDDLE = new Vector2d(50, 26);
        public final Vector2d SAMPLE_NEUTRAL_BLUE_CLOSE = new Vector2d(60, 26);

        public final Vector2d SAMPLE_RED_FAR = new Vector2d(40, -26);
        public final Vector2d SAMPLE_RED_MIDDLE = new Vector2d(50, -26);
        public final Vector2d SAMPLE_RED_CLOSE = new Vector2d(60, -26);

        public final Vector2d SAMPLE_BLUE_FAR = new Vector2d(-40, 26);
        public final Vector2d SAMPLE_BLUE_MIDDLE = new Vector2d(-50, 26);
        public final Vector2d SAMPLE_BLUE_CLOSE = new Vector2d(-60, 26);

        public final Vector2d SAMPLE_NEUTRAL_RED_FAR = new Vector2d(-40, -26);
        public final Vector2d SAMPLE_NEUTRAL_RED_MIDDLE = new Vector2d(-50, -26);
        public final Vector2d SAMPLE_NEUTRAL_RED_CLOSE = new Vector2d(-60, -26);
    }

    MecanumDrive.Params parameters = new MecanumDrive.Params();



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

        public Action closeClaw() {
            return new ClawClose();
        }

        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(parameters.CLAW_OPEN);
                return false;
            }
        }

        public Action openClaw() {
            return new ClawOpen();
        }
    }

    public class Twist {
        Servo twist;

        public Twist(HardwareMap hardwareMap) {
            Servo twist = hardwareMap.get(Servo.class, "twist");
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
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // Replace contents with whatever path you decide on in MeepMeep
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(45))
                .lineToX(5)
                .turn(Math.toRadians(90));

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("X Position during Init", drive.pose.position.x);
            telemetry.addData("Y Position during Init", drive.pose.position.y);
            telemetry.addData("Heading during Init", drive.pose.heading.real);

            telemetry.update();
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown()//,
                        //trajectoryActionCloseOut
                )
        );
    }
}
