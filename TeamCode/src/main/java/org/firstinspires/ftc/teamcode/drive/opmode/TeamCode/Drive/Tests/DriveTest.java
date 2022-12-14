package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;

@TeleOp(name = "DriveTest", group = "Tests")
public class DriveTest extends LinearOpMode {

    private Chassis Chassis;

    private GamepadEx Gamepad1;
    private GamepadEx Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Chassis = new Chassis(hardwareMap);

        // Initialize the gamepad
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        ToggleButtonReader chassisBrakeToggle = new ToggleButtonReader(
                Gamepad1,
                GamepadKeys.Button.Y
        );

        waitForStart();

        while (opModeIsActive()) {
            // Toggles the chassis brakes
            Chassis.toggleBrake(chassisBrakeToggle.getState());
            chassisBrakeToggle.readValue();

            // Updates the chassis speed based on gamepad1 bumpers
            Chassis.updateSpeed(
                    Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER),
                    Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
            );
//
//            // Drives the robot with joysticks from gamepad 1
//            Chassis.updateWithControls(
//                    Gamepad1.getLeftY(), //drive stick
//                    Gamepad1.getLeftX(), //strafe stick
//                    Gamepad1.getRightX() //turn stick
//            );
        }
    }
}
