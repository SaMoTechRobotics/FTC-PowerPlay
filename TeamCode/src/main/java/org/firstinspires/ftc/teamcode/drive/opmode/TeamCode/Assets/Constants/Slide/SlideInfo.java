package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
/**
 * Constant values for the slide
 * Height is from bottom to top of slide
 * All measurements are in inches 
*/
public class SlideInfo {
    /**
     * The max and min ticks of the slide
     * The max is the ticks of the motor when slide is fully extended
    */
    public static double MaxTicks = -2000.0;
    public static double MinTicks = 0.0;
    
    /**
     * The max and min heights of the slide
     * The max is the height of the slide when it is fully extended
    */
    public static double MaxHeight = 18.0;
    public static double MinHeight = 5.0;

}