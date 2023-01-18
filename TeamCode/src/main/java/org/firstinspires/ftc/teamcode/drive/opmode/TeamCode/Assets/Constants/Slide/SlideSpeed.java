package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide;

import com.acmerobotics.dashboard.config.Config;


@Config
/**
 * Constant values for the different speeds of the slide
 * 0.0 = no power
 * 1.0 = full power
*/
public class SlideSpeed {
    /**
     * The max, mid, and min speeds of the slide
     */
    public static double Max = 1.0;
    public static double Mid = 0.5;
    public static double Min = 0.1;

    /**
     * The speed of the slide to hold its position
     */
    public static double Hold = 0.2;

    /**
     * The speed of the slide when it is fully stopped
     */
    public static double Stop = 0.0;
}
