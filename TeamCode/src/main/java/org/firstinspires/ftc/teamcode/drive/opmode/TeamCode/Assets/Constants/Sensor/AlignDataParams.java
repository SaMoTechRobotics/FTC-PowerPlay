package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AlignDataParams {
    public static double OutlierMargin = 1.5; //the margin of error to remove an outlier
    public static int MinimumDataAmount = 3; //how long the robot has to be losing the pole to check if it is still losing the pole

    public static double LosingPoleMargin = 1; //if the robot is losing the pole check
    public static int LosingPoleDataAmount = 1; //how long (in terms of distances collected) the robot has to be losing the pole to check if it is still losing the pole

    public static double FindGiveUpTime = 5;

    public static double LeftForwardOffset = 0; //the amount the robot must drive forward to get the pole in the claw (left side)
    public static double RightForwardOffset = 1; //the amount the robot must drive forward to get the pole in the claw (right side)

}
