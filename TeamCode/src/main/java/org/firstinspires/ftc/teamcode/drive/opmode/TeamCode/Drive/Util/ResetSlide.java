package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;

@TeleOp(name = "Reset Slide", group = "A")
public class ResetSlide extends LinearOpMode {
    public static double SpeedModifier = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {


        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));
        Slide.resetToZero();

        Claw Claw =
                new Claw(
                        hardwareMap.get(Servo.class, "claw"),
                        hardwareMap.get(DistanceSensor.class, "clawDistanceSensor"),
                        hardwareMap.get(Servo.class, "poleBrace")
                );
        Claw.close();

        Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));

        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        telemetry.addLine("This opmode will allow you to reset the slide with the left joystick.");
        telemetry.addLine("Be careful, the slide can go below 0 and possibly damage string");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addLine("Use Left Joystick on Gamepad 2 to move Slide");
            telemetry.addLine("");

            Slide.setPower(Gamepad2.getLeftY() * SpeedModifier);


            telemetry.addData("Slide Power", Slide.getPower());
            telemetry.addLine("");

            telemetry.addLine("Press A to reset slide to zero");
            telemetry.addLine("");

            if (Gamepad2.wasJustPressed(GamepadKeys.Button.A)) {
                Slide.resetToZero();
            }
            if (Gamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
                Arm.setRotation(ArmRotation.Center);
            }
            if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                Claw.open();
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                Claw.close();
            }

            Gamepad2.readButtons();

            telemetry.addData("Slide Height", Slide.getInches());
            telemetry.update();
        }
    }
}
