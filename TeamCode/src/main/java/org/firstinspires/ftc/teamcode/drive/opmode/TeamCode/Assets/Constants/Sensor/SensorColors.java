package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This class contains constants for the color sensor as well as parking positions based on color
 */
@Config
public class SensorColors {

    /**
     * The detected color from the color sensor
     *
     * @param sensor the color sensor
     * @return the detected color as Color enum
     */
    public static Color detectColor(ColorSensor sensor) {
        return detectColor(sensor.red(), sensor.green(), sensor.blue()); // Detect the color based on the RGB values from the sensor
    }

    /**
     * The detected color from the values of r, g, b, checks to see which color value is the greatest
     *
     * @param r the red value
     * @param g the green value
     * @param b the blue value
     * @return the detected color as Color enum
     */
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

    /**
     * The parking position based on the detected color
     *
     * @param color the detected color
     * @return the parking position as an int, either 1, 2, or 3 (0 if error)
     */
    public static int getParkingPosition(Color color) {
        switch (color) { //Switches the color to match which parking spot it corresponds to
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

    /**
     * The color enum, used for detecting the color from the color sensor
     */
    public enum Color {
        Red, // Color of 1st parking position & red alliance cones
        Green, // Color of 2nd parking position
        Blue, // Color of 3rd parking position & blue alliance cones
        Grey, // Color of the grey mats
        Unknown // Unknown color
    }
}
