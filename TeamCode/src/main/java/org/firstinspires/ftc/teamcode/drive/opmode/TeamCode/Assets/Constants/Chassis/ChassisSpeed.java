package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
/**
 * Constant values for the different speeds of chassis
 * 0.0 = no power
 * 1.0 = full power
 */
public class ChassisSpeed {
    /**
     * The speed of the robot when it is moving forward or backward
     */
    public static double MaxDrive = 1.0;
    public static double MidDrive = 0.5;
    public static double MinDrive = 0.3;

    /**
     * The speed of the robot when it is turning
     */
    public static double MaxTurn = 1.0;
    public static double MidTurn = 0.5;
    public static double MinTurn = 0.3;

    /**
     * The speed of the robot when it is strafing
     */
    public static double MaxStrafe = 1.0;
    public static double MidStrafe = 0.5;
    public static double MinStrafe = 0.3;

    /**
     * The max and min speeds of the chassis motors
     */
    public static final double Max = 1.0;
    public static final double Min = 0.0;

    /**
     * Place speed for when robot is aligning with pole
     */
    public static double AlignSpeed = 0.15;

    /**
     * Place speed for when robot is driving toward pole to place cone
     */
    public static double PlaceSpeed = 0.1;


}
