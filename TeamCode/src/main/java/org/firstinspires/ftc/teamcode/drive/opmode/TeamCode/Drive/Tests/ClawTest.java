package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ClawTest", group = "Tests")
public class ClawTest extends LinearOpMode {

  private Arm Arm;
  private Claw Claw;

  private GamepadEx Gamepad1;
  private GamepadEx Gamepad2;

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    // Initialize the arm
    Arm = new Arm(hardwareMap.get(Servo.class, "arm"));

    // Initialize the claw
    Claw =
      new Claw(
        hardwareMap.get(Servo.class, "claw"),
        hardwareMap.get(DistanceSensor.class, "clawDistanceSensor")
      );

    // Initialize the gamepad
    Gamepad1 = new GamepadEx(gamepad1);
    Gamepad2 = new GamepadEx(gamepad2);

    ToggleButtonReader clawToggleButton = new ToggleButtonReader(
      Gamepad2,
      GamepadKeys.Button.RIGHT_BUMPER
    ); // The button to toggle the claw, A

    waitForStart();

    while (opModeIsActive()) {
      if (clawToggleButton.getState()) {
        // if toggle state true
        Claw.open();
      } else {
        // if toggle state false
        Claw.close();
      }
      clawToggleButton.readValue();

      Arm.updateWithControls(
        Gamepad2.getRightX(),
        gamepad2.x,
        gamepad2.b,
        gamepad2.y
      );
    }
  }
}
