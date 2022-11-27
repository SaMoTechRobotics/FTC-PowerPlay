package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Eyes;

import com.acmerobotics.dashboard.config.Config;

/**
 * This class contains constants for the eye servo
 */
@Config
public class EyesPosition {
    /**
     * The max position of the eye servo
     */
    public static double Max = 1.0;

    /**
     * The min position of the eye servo
     */
    public static double Min = 0.0;

    /**
     * The positions of the eye servo so that the eyes look straight forward
     */
    public static double Forward = 0.1;

    /**
     * The positions of the eye servo so that the eyes look down at the slide when the slide is all the way down
     */
    public static double SlideBase = 0;

    /**
     * The max position of the eye servo for it to be facing up
     */
    public static double MaxUp = 0.3;

    /**
     * The ratio of the servo position to the slide ticks
     */
    public static double SlideRatio = 0.001;
}
