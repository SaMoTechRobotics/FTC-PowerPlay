package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Camera;

@TeleOp(name = "CameraTest", group = "Tests")
public class CameraTest extends LinearOpMode {

    private Camera Camera;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Camera = new Camera(hardwareMap, dashboard);

        waitForStart();

        while (opModeIsActive()) {
            Camera.update(telemetry);
        }
    }
}
