package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;

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
                        hardwareMap.get(DistanceSensor.class, "leftDistanceSensor"),
                        hardwareMap.get(Servo.class, "poleBrace")
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
//            if (clawToggleButton.getState()) {
//                // if toggle state true
//                Claw.open();
//            } else {
//                // if toggle state false
//                Claw.close();
//            }
//            clawToggleButton.readValue();
            if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                Claw.close();
            } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                Claw.open();
            }

            Arm.updateWithControls(
                    Gamepad2.getRightX(),
                    gamepad2.x,
                    gamepad2.b,
                    gamepad2.y,
                    Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5
            );
        }
    }
}
