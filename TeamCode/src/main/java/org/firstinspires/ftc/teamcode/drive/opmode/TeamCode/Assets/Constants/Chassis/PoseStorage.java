package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * PoseStorage class which stores the pose of the robot between opmodes
 */
@Config
public class PoseStorage {
    /**
     * Current pose of the robot, saved between opmodes
     */
    public static Pose2d CurrentPose = new Pose2d();

    /**
     * Resets the position to 0, 0, 0
     */
    public static void reset() {
        CurrentPose = new Pose2d();
    }
}