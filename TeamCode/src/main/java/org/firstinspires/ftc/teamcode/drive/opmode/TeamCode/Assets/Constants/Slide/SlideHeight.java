package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideHeight {
    /**
     * The ticks per revolution of the slide motor
     */
    public static final double TicksPerRev = 384.5; // 537.7
    /**
     * The diameter of the pulley on the slide in inches
     */
    public static final double RevDiameter = 2.12598; //1.673228;
    /**
     * The ticks per inch of the slide moving up
     */
    public static final double TicksPerInch = TicksPerRev / (RevDiameter * Math.PI);
    public static boolean WaitForArm = true;
    /**
     * Heights of slide to line up with heights of different poles or ground as inches
     */
    public static double HighPole = 41;
    public static double MidPole = 30;
    public static double LowPole = 18;

    public static double Ground = 0;

    /**
     * Ticks of margin to consider the slide at 0
     */
    public static int GroundMargin = 10;

    public static int AutoClawMargin = 100;

    public static int GoingDownGroundMargin = 70;

    public static double PlaceMargin = 3;
    public static double[] StackConeHeights = {0, 1.6, 3, 4.2, 6};
    public static double StackConeHeightMultiplier = 1.2;
    /**
     * Max and min heights of slide as percentages
     */
    public static double MaxHeight = 42;
    public static double MinHeight = 0;
    /**
     * The base offset height, where slide is mounted relative to ground
     */
    public static double BaseHeight = 0;
    /**
     * The minimum height for the arm to be allowed to rotate
     */
    public static double SafetyHeight = 14;
    /**
     * The safety margin to move arm to center preventing failure in robot
     */
    public static double SafetyMargin = 15;

    public static double ClawOpenMargin = 18; //13

    public static double PoleBraceSafetyHeight = 20;
}
