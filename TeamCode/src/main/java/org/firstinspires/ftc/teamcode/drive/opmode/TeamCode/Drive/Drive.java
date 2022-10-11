package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {

    private Chassis Chassis;
    private Slide Slide;
    private Claw Claw;

    private GampepadEx Gamepad1;
    private GamepadEx Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Chassis = new Chassis(
        // hardwareMap.get(DcMotor.class, "frontLeft"),
        // hardwareMap.get(DcMotor.class, "frontRight"),
        // hardwareMap.get(DcMotor.class, "backLeft"),
        // hardwareMap.get(DcMotor.class, "backRight")
        // );

        // Initialize the slide
        Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        // Initialize the claw
        Claw = new Claw(hardwareMap.get(Servo.class, "claw"));

        // Initialize the gamepad
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);
            

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
            }

            if (gamepad1.dpad_down) {
                Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Max);
            }

            if (gamepad1.dpad_left) {
                Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Max);
            }

            if (gamepad1.dpad_right) {
                Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
            }

        }
    }
}