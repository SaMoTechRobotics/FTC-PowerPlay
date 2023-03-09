package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;

//@TeleOp(name = "Open Claw", group = "C")
public class OpenClaw extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Claw Claw =
                new Claw(
                        hardwareMap.get(Servo.class, "claw"),
                        hardwareMap.get(DistanceSensor.class, "clawDistanceSensor"),
                        hardwareMap.get(Servo.class, "poleBrace")
                );
        Claw.setOpenAmount(ClawPosition.PickupOpen);
        Claw.open();

        GamepadEx Gamepad2 = new GamepadEx(gamepad2);


        waitForStart();

        if (isStopRequested()) return;

        Claw.close();

        while (opModeIsActive()) {

            if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                Claw.open();
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                Claw.close();
            }
            Gamepad2.readButtons();

        }
    }
}
