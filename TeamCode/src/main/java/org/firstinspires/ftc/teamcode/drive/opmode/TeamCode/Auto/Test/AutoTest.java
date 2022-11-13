package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoTest", group = "Test")
public class AutoTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    AutoChassis Chassis = new AutoChassis(hardwareMap);

    waitForStart();

    if (isStopRequested()) return;

    Chassis
      .forward(20.0)
      .runBackgroundTask(() -> {
        telemetry.addData("Position", Chassis.getPosition().getX());
        telemetry.update();
      })
      .run();

    Chassis
      .turnRight(90.0)
      .run();

    Chassis
      .strafeRight(10.0)
      .setSpeed(0.1)
      .run();
  }
}
