package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Field;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Field.PoleHeight;

public class Field {

    public static Pole[][] Rows = {
            {new Pole(0, 0, PoleHeight.Ground), new Pole(1, 0, PoleHeight.Low)}
    };

    public static Pole getPole(int row, int column) {
        return Rows[row][column];
    }
}
