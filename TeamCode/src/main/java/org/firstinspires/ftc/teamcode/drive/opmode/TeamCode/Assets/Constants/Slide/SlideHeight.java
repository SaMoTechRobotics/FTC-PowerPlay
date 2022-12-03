package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideHeight {
    /**
     * The ticks per revolution of the slide motor
     */
    public static final double TicksPerRev = 537.7;
    /**
     * The diameter of the pulley on the slide in inches
     */
    public static final double RevDiameter = 1.673228;
    /**
     * The ticks per inch of the slide moving up
     */
    public static final double TicksPerInch = TicksPerRev / (RevDiameter * Math.PI);
    public static boolean WaitForArm = true;
    /**
     * Heights of slide to line up with heights of different poles or ground as inches
     */
    public static double HighPole = 43; //4000
    public static double MidPole = 31; //2800
    public static double LowPole = 18; //2000
    public static double Ground = 0;
    /**
     * Ticks of margin to consider the slide at 0
     */
    public static int GroundMargin = 10;
    public static double PlaceMargin = 3;
    public static double StackConeHeight = 1.0;
    /**
     * Max and min heights of slide as percentages
     */
    public static double MaxHeight = 45;
    public static double MinHeight = 0;
    /**
     * The base offset height, where slide is mounted relative to ground
     */
    public static double BaseHeight = 0;
    /**
     * The minimum height for the arm to be allowed to rotate
     */
    public static double SafetyHeight = 13;
    /**
     * The safety margin to move arm to center preventing failure in robot
     */
    public static double SafetyMargin = 15;
}
