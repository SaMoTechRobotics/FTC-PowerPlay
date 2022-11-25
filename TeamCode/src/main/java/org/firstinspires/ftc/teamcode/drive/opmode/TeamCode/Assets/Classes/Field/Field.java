package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Field;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Field.PoleHeight;

/**
 * This class contains the information about the field and stores it in a 2D array of Pole objects
 */
public class Field {
    /**
     * The size of each tile in inches
     */
    public static double TileSize = 24;

    /**
     * The field is a 2D array of poles that represent the field.
     * It is 5 rows with 5 poles in each row
     */
    public static Pole[][] Rows = {
            {new Pole(0, 0, PoleHeight.Ground), new Pole(0, 1, PoleHeight.Low), new Pole(0, 2, PoleHeight.Ground), new Pole(0, 3, PoleHeight.Low), new Pole(0, 4, PoleHeight.Ground)}, // Row 0, poles are: Ground, Low, Ground, Low, Ground
            {new Pole(1, 0, PoleHeight.Low), new Pole(1, 1, PoleHeight.Mid), new Pole(1, 2, PoleHeight.High), new Pole(1, 3, PoleHeight.Mid), new Pole(1, 4, PoleHeight.Low)}, // Row 1, poles are: Low, Mid, High, Mid, Low
            {new Pole(2, 0, PoleHeight.Ground), new Pole(2, 1, PoleHeight.High), new Pole(2, 2, PoleHeight.Ground), new Pole(2, 3, PoleHeight.High), new Pole(2, 4, PoleHeight.Ground)}, // Row 2, poles are: Ground, High, Ground, High, Ground
            {new Pole(1, 0, PoleHeight.Low), new Pole(1, 1, PoleHeight.Mid), new Pole(1, 2, PoleHeight.High), new Pole(1, 3, PoleHeight.Mid), new Pole(1, 4, PoleHeight.Low)}, // Row 3, poles are: Low, Mid, High, Mid, Low
            {new Pole(0, 0, PoleHeight.Ground), new Pole(0, 1, PoleHeight.Low), new Pole(0, 2, PoleHeight.Ground), new Pole(0, 3, PoleHeight.Low), new Pole(0, 4, PoleHeight.Ground)}, // Row 4, poles are: Ground, Low, Ground, Low, Ground
    };

    /**
     * Gets the pole based off the row and column pole is in
     *
     * @param row    The row the pole is in
     * @param column The column the pole is in
     * @return The pole at the row and column
     */
    public static Pole getPole(int row, int column) {
        return Rows[row][column]; // Gets the pole from the row array
    }

    /**
     * Gets the current row that the robot is in
     *
     * @param pose The current pose of the robot
     * @return The row the robot is in
     */
    public static int getCurrentRow(Pose2d pose) {
        return (int) Math.round((pose.getX() / TileSize) - 0.5);
    }

    /**
     * Gets the current column that the robot is in
     *
     * @param pose The current pose of the robot
     * @return The column the robot is in
     */
    public static int getCurrentColumn(Pose2d pose) {
        return (int) Math.round((pose.getY() / TileSize) - 0.5);
    }
}
