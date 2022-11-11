package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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
        hardwareMap.get(DcMotor.class, "backRight"),
        hardwareMap
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
    ); // The button that toggles the chassis brake

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

      Chassis.updatePosition();

      Pose2d ChassisPos = Chassis.getPosition();
      telemetry.addData("","");
      telemetry.addData("Chassis X", ChassisPos.getX());
      telemetry.addData("Chassis Y", ChassisPos.getY());
      telemetry.addData("Chassis Heading", ChassisPos.getHeading());

      /* CLAW */

      Claw.toggleOpen(clawToggleButton.getState());
      clawToggleButton.readValue();

      telemetry.addData("","");
      telemetry.addData("Claw Position", Claw.getPosition());

      /* ARM */

      if(Slide.getInches() > SlideHeight.SafetyHeight) Arm.updateWithControls(
        Gamepad2.getRightX(),
        gamepad2.x,
        gamepad2.b,
        gamepad2.y
      );

      telemetry.addData("","");
      telemetry.addData("Arm Position", Claw.getPosition());

      /* SLIDE */

      Slide.updateWithControls(
        Gamepad2.getLeftY(),
        Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_UP),
        Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT),
        Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
        Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)
      );

      Slide.updateSpeed(
        Gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)
      );

      telemetry.addData("","");
      telemetry.addData("Slide Inches", Slide.getInches());
      telemetry.addData("Slide Ticks", Slide.getTicks());
      telemetry.addData("Slide Status", Slide.getStatus());

      /* GENERAL */

      // if (Slide.getInches() < SlideHeight.SafetyMargin) { //make sure this only happens once
      //   Arm.setRotation(ArmRotation.Center);
      //   // if(Slide.getInches() > SlideHeight.SafetyHeight) Claw.close();
      // }
      // if (Slide.getInches() < SlideHeight.SafetyHeight && Arm.getRotation() != ArmRotation.Center) {
      //   Slide.setPower(0);
      // } else {
      //   // Slide.setPower(Slide.Speed);
      // }

      // if(Slide.getInches() < SlideHeight.SafetyHeight && previousSlideHeight >= SlideHeight.SafetyHeight) {
      //   Arm.setRotation(ArmRotation.Center);
      //   // Claw.close();
      // }

      if(Slide.getInches() < SlideHeight.SafetyMargin) {
        Arm.setRotation(ArmRotation.Center);
      }
      if(Slide.getInches() < SlideHeight.SafetyHeight && Arm.getRotation() != ArmRotation.Center) {
        Slide.pause();
      } else {
        if(Slide.isPaused) Slide.resume();
      }

      Gamepad1.readButtons();
      Gamepad2.readButtons();
      telemetry.update();
    }
  }
}
