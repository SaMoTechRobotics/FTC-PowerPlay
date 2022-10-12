package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmPosition {
    
    /**
     * Max and min rotations of the arm, ex: -90.0 to 90.0 degrees resulting in 180 degree range
     */
    public static double Min = -90.0;
    public static double Max = 90.0;
    
    /**
     * The full rotations of the arm to the left and right
     */
    public static double FullLeft = -90.0;
    public static double FullRight = 90.0;

    /**
     * The half rotations of the arm to the left and right
     */
    public static double Left = -45.0;
    public static double Right = 45.0;

    /**
     * The default/center/initial rotation of the arm
     */
    public static double Center = 0.0;
}