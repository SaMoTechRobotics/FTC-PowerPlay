package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SensorColors {

    public static enum Color {
        Red,
        Green,
        Blue,
        Grey,
        Unknown
    }

    public static Color detectColor(int r, int g, int b) {
        if (r > g && r > b) {
            return Color.Red;
        } else if (g > r && g > b) {
            return Color.Green;
        } else if (b > r && b > g) {
            return Color.Blue;
        } else if (r == g && g == b) {
            return Color.Grey;
        } else {
            return Color.Unknown;
        }

    }
}
