package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class TrajSpeed {
    public double Accel;
    public double Speed;
    public double Turn;

    public TrajSpeed(double accel, double speed, double... turn) {
        Accel = accel;
        Speed = speed;
        if (turn.length > 0) Turn = turn[0];
        else Turn = DriveConstants.MAX_ANG_VEL_DEG;
    }
}
