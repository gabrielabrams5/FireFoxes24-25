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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED_DIVE_AUTONOMOUS", group = "Autonomous")
public class RedDiveAutonomous extends Autonomous{


    @Override
    public void runOpMode() {
        Autonomous.StartingPosition startPos = StartingPosition.RED_DIVE;

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

        Actions.runBlocking(
                new ParallelAction(
                        robot.launch.moveLaunch(),
                        robot.intake.moveIntake(),
                        actionToExecute
                )
        );
    }
}
