package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideHeight {
    /**
     * Heights of slide to line up with heights of different poles or ground as inches
     */
    public static double HighPole = 42; //4000
    public static double MidPole = 32; //2800
    public static double LowPole = 22; //2000
    public static double Ground = 0;

    /**
     * Max and min heights of slide as percentages
     */
    public static double MaxHeight = 42;
    public static double MinHeight = 0;

    /**
     * The base offset height, where slide is mounted relative to ground
     */
    public static double BaseHeight = 4;

    /**
     * The safety margin to move arm to center preventing failure in robot
     */
    public static double SafetyHeight = 18;

    public static double SafetyMargin = 18;


    /**
     * Max and min ticks of slide motor
     */
    public static double MaxTicks = 4000.0;
    public static double MinTicks = 0.0;

    /**
     * The ticks per revolution of the slide motor
     */
    public static double TicksPerRev = 537.7;

    /**
     * The diameter of the pulley on the slide in inches
     */
    public static double RevDiameter = 1.673228;

    /**
     * The ticks per inch of the slide moving up
     */
    public static double TicksPerInch = TicksPerRev / (RevDiameter * Math.PI);
}