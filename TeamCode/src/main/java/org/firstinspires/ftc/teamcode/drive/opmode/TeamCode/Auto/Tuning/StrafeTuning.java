package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "StrafeTuning", group = "AutoTest")
public class StrafeTuning extends LinearOpMode {

  public static double strafeAmount = 20.0;

  @Override
  public void runOpMode() throws InterruptedException {
    AutoChassis Chassis = new AutoChassis(hardwareMap);

    waitForStart();

    if (isStopRequested()) return;

    Chassis.strafeRight(strafeAmount).run();

    while (opModeIsActive()) {
      ChassisPosition position = Chassis.getPosition();
      telemetry.addData("X", position.getX());
      telemetry.addData("Y", position.getY());
      telemetry.addData("Rotation", position.getRotation());
      telemetry.update();
    }
  }
}
