package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
