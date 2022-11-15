package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "TurnTuning", group = "AutoTest")
public class TurnTuning extends LinearOpMode {

  public static double turnAmount = 90.0;

  @Override
  public void runOpMode() throws InterruptedException {
    AutoChassis Chassis = new AutoChassis(hardwareMap);

    waitForStart();

    if (isStopRequested()) return;

    Chassis.turnRight(turnAmount).run();

    while (opModeIsActive()) {
      ChassisPosition position = Chassis.getPosition();
      telemetry.addData("X", position.getX());
      telemetry.addData("Y", position.getY());
      telemetry.addData("Rotation", position.getRotation());
      telemetry.update();
    }
  }
}
