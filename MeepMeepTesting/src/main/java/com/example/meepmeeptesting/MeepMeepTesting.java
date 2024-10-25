package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

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

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d bucketPose = new Pose2d(55, 54, Math.toRadians(45));
        double secondsToWait = 1;

        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        // Blue close
        Action blueBucket = robot.getDrive().actionBuilder(new Pose2d(35, 62, 0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(315))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_FAR, Math.toRadians(-80))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE, Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_CLOSE, Math.toRadians(-60))
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(26, 10, Math.toRadians(180)), Math.toRadians(270))
                .build();

        robot.runAction(blueBucket);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}