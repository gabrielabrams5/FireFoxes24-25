package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED_BUCKET_AUTONOMOUS", group = "Autonomous")
public class RedBucketAutonomous extends Autonomous{


    @Override
    public void runOpMode() {
        Autonomous.StartingPosition startPos = StartingPosition.RED_BUCKET;

        Pose2d initialPose = startPos.getStartPos();
        Autonomous.Robot robot = new Robot(
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
                        robot.launch.moveLaunch(),
                        robot.intake.moveIntake(),
                        actionToExecute
                )
        );
    }
}
