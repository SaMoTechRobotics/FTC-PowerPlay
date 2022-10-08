package org.firstinspires.ftc.teamcode.drive.opmode;

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
    static double MaxDrive = 1.0;
    static double MidDrive = 0.6;
    static double MinDrive = 0.3;

    /**
     * The speed of the robot when it is turning
    */
    static double MaxTurn = 1.0;
    static double MidTurn = 0.6;
    static double MinTurn = 0.3;

    /**
     * The speed of the robot when it is strafing
    */
    static double MaxStrafe = 1.0;
    static double MidStrafe = 0.6;
    static double MinStrafe = 0.3;

    /**
     * The max and min speeds of the chassis motors
    */
    public static double Max = 1.0;
    public static double Min = 0.0;
}
