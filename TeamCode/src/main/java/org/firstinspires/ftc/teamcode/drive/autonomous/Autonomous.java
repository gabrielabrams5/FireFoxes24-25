package org.firstinspires.ftc.teamcode.drive.autonomous;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
    public class Lift {
        private final DcMotorEx linearSlide1;
        private final DcMotorEx linearSlide2;

        public Lift(HardwareMap hardwareMap) {
            linearSlide1 = hardwareMap.get(DcMotorEx.class, "linearSlide1");
            linearSlide2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");
            linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
            linearSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
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
                packet.put("liftPos", pos);
                if (pos < 3500.0) {  // 3500.0 is the LINEAR_SLIDE_MAX in drive.java, adjust here if adjusted elsewhere
                    // Keep raising lift if it hasn't reached max height yet
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
                if (pos > 200.0) {  // 200.0 is the LINEAR_SLIDE_MIN in drive.java, adjust here if adjusted elsewhere
                    // Keep lowering lift if it hasn't reached max height yet
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

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.45);  // Hardcoded from drive.java
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(.75);  // Also hardcoded
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
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
                twist.setPosition(0.0);
                return false;
            }
        }

        public Action twistUp() {
            return new TwistUp();
        }

        public class TwistDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                twist.setPosition(0.75);
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
    }
}
