package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.PoseStorage;

@TeleOp(name = "Reset Pose", group = "Util")
public class ResetPose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Ready to reset pose");
        telemetry.addData("Current Pose", PoseStorage.CurrentPose);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        PoseStorage.reset();

        telemetry.addData("Status", "Reset pose complete");
        telemetry.addData("Current Pose", PoseStorage.CurrentPose);
        telemetry.update();
    }
}
