package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for sensor distances, used for autonomous and auto alignment
 */
@Config
public class SensorDistances {

    /**
     * The maximum distance the robot will detect a pole
     */
    public static double DetectAmount = 12;

    /**
     * The target distance the robot will try to align to to deliver the cone
     */
    public static double PlaceDistance = 4.5;

    /**
     * The margin of error for the robot to be within the target distance
     */
    public static double PlaceMargin = 0.25;

    /**
     * The target distance the robot will try to align to where it is in between the two poles equally
     */
    public static double CenterDistance = 7;

    /**
     * The buffer for how long the robot will drive until it starts going back
     */
    public static double FindBuffer = 5;
}
