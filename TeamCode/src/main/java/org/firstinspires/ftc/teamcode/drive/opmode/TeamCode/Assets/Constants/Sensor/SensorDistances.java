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
    public static double DetectAmount = 9.5;

    public static double ManualDetectAmount = 12;

    /**
     * The target distance the robot will try to align to to deliver the cone
     */
    public static double LeftPlaceDistance = 2.5;
    public static double RightPlaceDistance = 3.2;


    public static double LeftCenterDistance = 5;
    public static double RightCenterDistance = 5;

    public static double DriveBackAdjust = 1;

    public static double ManualPlaceDistance = 4.75;

    /**
     * The margin of error for the robot to be within the target distance
     */
    public static double PlaceMargin = 0.25;

    public static double OutlierMargin = 2; //the margin of error to remove an outlier

    public static double LosingPoleMargin = 1; //if the robot is losing the pole check
    public static double StillLosingPoleMargin = 1; //if the robot is still losing the pole check

    public static int MinimumDataAmount = 3; //how long the robot has to be losing the pole to check if it is still losing the pole

    public static double ManualPlaceMargin = 0.25;

    /**
     * The target distance the robot will try to align to where it is in between the two poles equally
     */
    public static double CenterDistance = 7;

    /**
     * The buffer for how long the robot will drive until it starts going back
     */
    public static double FindBuffer = 3;

    public static double FindGiveUpTime = 5;
}
