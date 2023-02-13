package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawPosition {
    public static boolean AutoClose = true;

    //open: 40, close: 90
    /**
     * Max and min positions of the claw, ex: 0.0 to 1.0 resulting in 1.0 range
     */
    public static double Max = 1.0;
    public static double Min = 0.0;

    /**
     * The max open and close positions of the claw
     */
    public static double MaxOpen = 100;
    public static double MaxClosed = 0;

    /**
     * The open and close positions of the claw
     */
    public static double Open = 40;
    public static double PickupOpen = 68; //old claw: 86
    public static double Close = 92; //old claw: 70

    public static double ConeDistance = 2;

    public static double ResetOpenMargin = 20;

    public static double PoleBraceUp = 0.9;


    public static double PoleBraceDown = 0.5;
    public static double PoleBraceDownRight = 0.1;
    public static double PoleBraceDownLeft = 0.17;

    public static double PoleBraceDownSensor = 0.5;

    public static boolean ReversePoleBrace = false;

    public enum PoleBraceAlignDirection {
        Forward,
        Backward,
    }
}


