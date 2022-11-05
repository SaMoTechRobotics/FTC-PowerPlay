package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive", group = "Drive")
public class Drive extends LinearOpMode {

  private Chassis Chassis;
  private Slide Slide;
  private Arm Arm;
  private Claw Claw;

  private GamepadEx Gamepad1;
  private GamepadEx Gamepad2;

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    Chassis =
      new Chassis(
        hardwareMap.get(DcMotor.class, "frontLeft"),
        hardwareMap.get(DcMotor.class, "frontRight"),
        hardwareMap.get(DcMotor.class, "backLeft"),
        hardwareMap.get(DcMotor.class, "backRight")
      );

    Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

    Arm = new Arm(hardwareMap.get(Servo.class, "arm"));

    Claw = new Claw(hardwareMap.get(Servo.class, "claw"));

    // Initialize the gamepad
    Gamepad1 = new GamepadEx(gamepad1);
    Gamepad2 = new GamepadEx(gamepad2);

    ToggleButtonReader chassisBrakeToggle = new ToggleButtonReader(
      Gamepad1,
      GamepadKeys.Button.Y
    );

    ToggleButtonReader clawToggleButton = new ToggleButtonReader(
        Gamepad2,
        GamepadKeys.Button.RIGHT_BUMPER
      ); // The button to toggle the claw

    waitForStart();

    while (opModeIsActive()) {
      /* CHASSIS */

      // Toggles the chassis brakes
      Chassis.toggleBrake(chassisBrakeToggle.getState());
      chassisBrakeToggle.readValue();

      // Updates the chassis speed based on gamepad1 bumpers
      Chassis.updateSpeed(
        Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER),
        Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
      );

      // Drives the robot with joysticks from gamepad 1
      Chassis.updateWithControls(
        Gamepad1.getLeftY(), //drive stick
        Gamepad1.getLeftX(), //strafe stick
        Gamepad1.getRightX() //turn stick
      );

      /* CLAW */

      Claw.toggleOpen(clawToggleButton.getState());
      clawToggleButton.readValue();

      /* ARM */

      Arm.updateWithControls(
        Gamepad2.getRightX(),
        gamepad2.x,
        gamepad2.b,
        gamepad2.y
      );

      /* SLIDE */

      Slide.updateWithControls(
        Gamepad2.getLeftY(),
        gamepad2.dpad_up,
        gamepad2.dpad_left,
        gamepad2.dpad_down,
        gamepad2.dpad_right
    );

    }
  }
}
