package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawPosition {
    
    /**
     * Max and min positions of the claw, ex: 0.0 to 1.0 resulting in 1.0 range
     */
    public static double Max = 1.0;
    public static double Min = 0.0;

    /**
     * The max open and close positions of the claw
     */
    public static int MaxOpen = 100;
    public static int MaxClosed = 0;

    /**
     * The normal open and close positions of the claw
     */
    public static int Open = 90;
    public static int Closed = 40;

    
    
}