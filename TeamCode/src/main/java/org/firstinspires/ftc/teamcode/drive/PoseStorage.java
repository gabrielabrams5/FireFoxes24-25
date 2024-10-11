package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

public class PoseStorage {
    // See this static keyword? That's what lets us share the data between opmodes.
    public static Pose2d currentPose = new Pose2d(new Vector2d(0,0), new Rotation2d(0,0));
}
