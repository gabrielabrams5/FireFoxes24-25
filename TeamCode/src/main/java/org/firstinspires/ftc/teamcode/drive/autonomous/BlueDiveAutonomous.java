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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE_DIVE_AUTONOMOUS", group = "Autonomous")
public class BlueDiveAutonomous extends Autonomous{


    @Override
    public void runOpMode() {
        Autonomous.StartingPosition startPos = StartingPosition.BLUE_DIVE;

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
