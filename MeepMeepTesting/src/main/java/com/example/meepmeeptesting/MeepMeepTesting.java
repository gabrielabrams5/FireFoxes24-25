package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d bucketPose = new Pose2d(55, 54, Math.toRadians(45));
        double secondsToWait = 1;
        double initialBlockX = 35;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();
        // Blue close
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(35, 62, 0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(bucketPose, Math.toRadians(315))
                .waitSeconds(secondsToWait)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(initialBlockX+10*0, 26, Math.toRadians(0)), 0)
                .waitSeconds(secondsToWait)
                .splineToLinearHeading(bucketPose, 45)
                .waitSeconds(secondsToWait)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(initialBlockX+10*1, 26, Math.toRadians(0)), 0)
                .waitSeconds(secondsToWait)
                .splineToLinearHeading(bucketPose, 45)
                .waitSeconds(secondsToWait)
                .setTangent(Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(initialBlockX+10*2, 26, Math.toRadians(0)), 0)
                .waitSeconds(secondsToWait)
                .splineToLinearHeading(bucketPose, 45)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(26, 10, Math.toRadians(180)), Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}