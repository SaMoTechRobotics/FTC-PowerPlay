package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;


@Config
@Autonomous(name = "StraightTuning", group = "AutoTest")
public class StraightTuning extends LinearOpMode {

    public static double forwardAmount = 20.0;

  @Override
  public void runOpMode() throws InterruptedException {
    AutoChassis Chassis = new AutoChassis(hardwareMap);

    waitForStart();

    if (isStopRequested()) return;

    Chassis.forward(20.0).run();

    while (opModeIsActive()) {
      ChassisPosition position = Chassis.getPosition();
      telemetry.addData("X", position.getX());
      telemetry.addData("Y", position.getY());
      telemetry.addData("Rotation", position.getRotation());
      telemetry.update();
    }
  }
}
