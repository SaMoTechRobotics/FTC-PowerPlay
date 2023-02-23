package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor;

public class AlignDataParams {
    public static double OutlierMargin = 2; //the margin of error to remove an outlier
    public static int MinimumDataAmount = 3; //how long the robot has to be losing the pole to check if it is still losing the pole

    public static double LosingPoleMargin = 1; //if the robot is losing the pole check
    public static int LosingPoleDataAmount = 1; //how long (in terms of distances collected) the robot has to be losing the pole to check if it is still losing the pole

    public static double FindGiveUpTime = 5;

}
