package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * PoseStorage class which stores the pose of the robot between opmodes
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();

    public static void reset() {
        currentPose = new Pose2d();
    }
}