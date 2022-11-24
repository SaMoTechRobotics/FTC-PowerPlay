package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Field;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Field.PoleHeight;

public class Field {

    /**
     * The field is a 2D array of poles that represent the field.
     * It is 5 rows with 5 poles in each row
     */
    public static Pole[][] Rows = {
            {new Pole(0, 0, PoleHeight.Ground), new Pole(0, 1, PoleHeight.Low), new Pole(0, 2, PoleHeight.Ground), new Pole(0, 3, PoleHeight.Low), new Pole(0, 4, PoleHeight.Ground)},
            {new Pole(1, 0, PoleHeight.Low), new Pole(1, 1, PoleHeight.Mid), new Pole(1, 2, PoleHeight.High), new Pole(1, 3, PoleHeight.Mid), new Pole(1, 4, PoleHeight.Low)},
            {new Pole(2, 0, PoleHeight.Ground), new Pole(2, 1, PoleHeight.High), new Pole(2, 2, PoleHeight.Ground), new Pole(2, 3, PoleHeight.High), new Pole(2, 4, PoleHeight.Ground)},
            {new Pole(1, 0, PoleHeight.Low), new Pole(1, 1, PoleHeight.Mid), new Pole(1, 2, PoleHeight.High), new Pole(1, 3, PoleHeight.Mid), new Pole(1, 4, PoleHeight.Low)},
            {new Pole(0, 0, PoleHeight.Ground), new Pole(0, 1, PoleHeight.Low), new Pole(0, 2, PoleHeight.Ground), new Pole(0, 3, PoleHeight.Low), new Pole(0, 4, PoleHeight.Ground)},
    };

    public static Pole getPole(int row, int column) {
        return Rows[row][column];
    }
}
