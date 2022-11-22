package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Config
public class SensorColors {

    public static Color detectColor(ColorSensor sensor) {
        return detectColor(sensor.red(), sensor.green(), sensor.blue());
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

    public static int getParkingPosition(Color color) {
        switch (color) {
            case Red:
                return 1;
            case Green:
                return 2;
            case Blue:
                return 3;
            default:
                return 0;
        }
    }

    public enum Color {
        Red,
        Green,
        Blue,
        Grey,
        Unknown
    }
}
