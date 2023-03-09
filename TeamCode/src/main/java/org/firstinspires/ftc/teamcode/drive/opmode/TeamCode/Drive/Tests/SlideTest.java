package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;


//@TeleOp(name = "SlideTest", group = "Tests")
public class SlideTest extends LinearOpMode {

    private Slide Slide;

    private GamepadEx Gamepad1;
    private GamepadEx Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize the slide
        Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        // Initialize the gamepad
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);


        waitForStart();

        while (opModeIsActive()) {

            // Slide.updateWithControls(
            //     Gamepad2.getLeftY(),
            //     gamepad2.dpad_up,
            //     gamepad2.dpad_left,
            //     gamepad2.dpad_down,
            //     gamepad2.dpad_right
            // );

            telemetry.addData("Slide Ticks", Slide.getTicks());
            telemetry.addData("Slide Inches", Slide.getInches());
            telemetry.update();


        }
    }
}