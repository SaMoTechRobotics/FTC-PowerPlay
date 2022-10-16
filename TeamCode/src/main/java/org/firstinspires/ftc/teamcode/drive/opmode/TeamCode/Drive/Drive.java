package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {

    private Chassis Chassis;
    private Slide Slide;
    private Arm Arm;
    private Claw Claw;

    private GampepadEx Gamepad1;
    private GamepadEx Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Chassis = new Chassis(
            hardwareMap.get(DcMotor.class, "frontLeft"),
            hardwareMap.get(DcMotor.class, "frontRight"),
            hardwareMap.get(DcMotor.class, "backLeft"),
            hardwareMap.get(DcMotor.class, "backRight")
        );

        // Initialize the slide
        Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        // Initialize the arm
        Arm = new Arm(hardwareMap.get(Servo.class, "arm"));

        // Initialize the claw
        Claw = new Claw(hardwareMap.get(Servo.class, "claw"));

        // Initialize the gamepad
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        ToggleButtonReader clawToggleButton = new ToggleButtonReader(
                Gamepad2, GamepadKeys.Button.X);


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

            if(Gamepad1.getLeftBumper()) {
                Chassis.DriveSpeed = ChassisSpeed.MaxDrive;
                Chassis.TurnSpeed = ChassisSpeed.MaxTurn;
                Chassis.StrafeSpeed = ChassisSpeed.MaxStrafe;
            } else if(Gamepad1.getRightBumper()) {
                Chassis.DriveSpeed = ChassisSpeed.MidDrive;
                Chassis.TurnSpeed = ChassisSpeed.MidTurn;
                Chassis.StrafeSpeed = ChassisSpeed.MidStrafe;
            } else {
                Chassis.DriveSpeed = ChassisSpeed.MinDrive;
                Chassis.TurnSpeed = ChassisSpeed.MinTurn;
                Chassis.StrafeSpeed = ChassisSpeed.MinStrafe;
            }


            Chassis.updateControls(
                Gamepad1.getLeftStickY(),
                Gamepad1.getLeftStickX(),
                Gamepad1.getRightStickX()
            );



            if(gamepad2.left_y != 0) {
                Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Slide.setPower((gamepad2.left_y));
            }

            if (gamepad2.dpad_up) {
                Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
            }

            if (gamepad2.dpad_down) {
                Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Max);
            }

            if (gamepad2.dpad_left) {
                Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Max);
            }

            if (gamepad2.dpad_right) {
                Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
            }

        }
    }
}