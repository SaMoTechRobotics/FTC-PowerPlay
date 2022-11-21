package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Field;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Field.PoleHeight;

public class Pole {

    public int Row;
    public int Column;

    public PoleHeight Height = PoleHeight.Ground;

    public Pole(int row, int column, PoleHeight height) {
        this.Row = row;
        this.Column = column;

        this.Height = height;
    }
}
