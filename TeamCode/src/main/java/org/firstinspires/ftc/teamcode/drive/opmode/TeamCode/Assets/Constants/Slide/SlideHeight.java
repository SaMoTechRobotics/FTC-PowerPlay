package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideHeight {
    /**
     * Heights of slide to line up with heights of different poles or ground as inches
     */
    public static double HighPole = 20; //4000
    public static double MidPole = 18; //2800
    public static double LowPole = 10; //2000
    public static double Ground = 0;

    /**
     * Max and min heights of slide as percentages
     */
    public static double MaxHeight = 20;
    public static double MinHeight = 0;

    /**
     * The base offset height, where slide is mounted relative to ground
     */
    public static double BaseHeight = 2;

    /**
     * The safety margin to move arm to center preventing failure in robot
     */
    public static double SafetyHeight = 5;


    /**
     * Max and min ticks of slide motor
     */
    public static double MaxTicks = 4000.0;
    public static double MinTicks = 0.0;

    public static double TicksPerRev = 537.7;
    public static double TicksPerInch = TicksPerRev / (4 * Math.PI);
}