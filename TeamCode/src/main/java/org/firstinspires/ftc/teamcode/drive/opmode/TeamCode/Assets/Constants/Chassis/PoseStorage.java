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
    public static Pose2d currentPose = new Pose2d();

    /**
     * Starting pose on the left
     */
    public static Pose2d leftStartPose = new Pose2d(-60, -48, Math.toRadians(90));

    /**
     * Starting pose on the right
     */
    public static Pose2d rightStartPose = new Pose2d(-60, -48, Math.toRadians(90));

    /**
     * Resets the position to 0, 0, 0
     */
    public static void reset() {
        currentPose = new Pose2d();
    }
}