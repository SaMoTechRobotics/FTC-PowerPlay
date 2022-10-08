package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
/**
 * This class contains all the constants for the chassis
 * All measurements are in inches
*/
public static class ChassisInfo {
    /**
     * The full dimensions of the robot measured from outside to outside
     */
    public static double FullWidth = 18.0;
    public static double FullLength = 18.0;
    public static double FullHeight = 18.0;

    /**
     * Wheel diameter in inches
    */    
    public static double WheelDiameter = 4.0;

    /**
     * The circumference of the wheel in inches
     */
    public static double WheelCircumference = WheelDiameter * Math.PI;

    /**
     * The width of the wheel in inches
     */
    public static double WheelWidth = 1.0;

    /**
     * The ticks per revolution of the motor
    */
    public static double TicksPerRev = 537.6;

    /**
     * The distance between the center of the wheels
    */
    public static double InnerWidth = FullWidth - WheelWidth;
}
