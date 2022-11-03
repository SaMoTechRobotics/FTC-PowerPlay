package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRotation {
    
    /**
     * Max and min rotations of the arm, ex: -90.0 to 90.0 degrees resulting in 180 degree range
     */
    public static double Min = 0.0;
    public static double Max = 1.0;
    
    /**
     * The full rotations of the arm to the left and right
     */
    public static double FullLeft = 1.0; //1.0
    public static double FullRight = 0.0; //0.0

    /**
     * The half rotations of the arm to the left and right where arm is still within robot boundries
     */
    public static double Left = 0.8; //0.8
    public static double Right = 0.1; //0.1

    /**
     * The default/center/initial rotation of the arm
     */
    public static double Center = 0.45; //0.5
}