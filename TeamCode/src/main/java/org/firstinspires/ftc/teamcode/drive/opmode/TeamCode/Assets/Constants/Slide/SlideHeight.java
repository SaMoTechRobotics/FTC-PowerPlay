package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideHeight {
    /**
     * Heights of slide to line up with heights of different poles or ground as percentages
     */
    public static double HighPole = 100; //4000
    public static double MidPole = 70; //2800
    public static double LowPole = 50; //2000
    public static double Ground = 0;

    /**
     * Max and min heights of slide as percentages
     */
    public static double Max = 100;
    public static double Min = 0;


    /**
     * Max and min ticks of slide motor
     */
    public static double MaxTicks = 4000.0;
    public static double MinTicks = 0.0;
}