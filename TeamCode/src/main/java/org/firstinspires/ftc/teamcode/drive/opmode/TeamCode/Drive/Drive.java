package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;


@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {

    private Chassis Chassis;
    private Slide Slide;
    private Arm Arm;
    private Claw Claw;

    private GamepadEx Gamepad1;
    private GamepadEx Gamepad2;

    private StandardTrackingWheelLocalizer chassisLocalizer;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Chassis = new Chassis(
            hardwareMap.get(DcMotor.class, "frontLeft"),
            hardwareMap.get(DcMotor.class, "frontRight"),
            hardwareMap.get(DcMotor.class, "backLeft"),
            hardwareMap.get(DcMotor.class, "backRight")
        );

        // StandardTrackingWheelLocalizer chassisLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        // chassisLocalizer.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


        // Initialize the slide
        // Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        // Initialize the arm
        // Arm = new Arm(hardwareMap.get(Servo.class, "arm"));

        // Initialize the claw
        // Claw = new Claw(hardwareMap.get(Servo.class, "claw"));

        // Initialize the gamepad
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        // ToggleButtonReader clawToggleButton = new ToggleButtonReader(
        //         Gamepad2, GamepadKeys.Button.X); // The button to toggle the claw, X

        ToggleButtonReader chassisBrakeToggle = new ToggleButtonReader(
            Gamepad1, GamepadKeys.Button.Y
        );

        waitForStart();

        while (opModeIsActive()) {

            // chassisLocalizer.update();

            // Retrieve your pose
            // Pose2d chassisPose = chassisLocalizer.getPoseEstimate();

            telemetry.addData("x", chassisPose.getX());
            telemetry.addData("y", chassisPose.getY());
            telemetry.addData("heading", chassisPose.getHeading());

            // // Toggles the claw based of the clawToggleButton which is x
            // if (clawToggleButton.getState()) {
            //     // if toggle state true
            //     Claw.open();
            // } else {
            //     // if toggle state false
            //     Claw.close();
            // }
            // clawToggleButton.readValue();


            // Toggles the chassis braking on zero power with the Y button
            if(chassisBrakeToggle.getState()) {
                Chassis.toggleBrake(true);
            } else {
                Chassis.toggleBrake(false);
            }
            chassisBrakeToggle.readValue();

            /**
             * Toggles the speeds of driving with bumpers
             */
            if(Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER)) { // Left bumper is max speeds
                Chassis.DriveSpeed = ChassisSpeed.MaxDrive;
                Chassis.TurnSpeed = ChassisSpeed.MaxTurn;
                Chassis.StrafeSpeed = ChassisSpeed.MaxStrafe;
            } else if(Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) { // Right bumper is min speeds
                Chassis.DriveSpeed = ChassisSpeed.MinDrive;
                Chassis.TurnSpeed = ChassisSpeed.MinTurn;
                Chassis.StrafeSpeed = ChassisSpeed.MinStrafe;
            } else { // No bumper is mid speeds
                Chassis.DriveSpeed = ChassisSpeed.MidDrive;
                Chassis.TurnSpeed = ChassisSpeed.MidTurn;
                Chassis.StrafeSpeed = ChassisSpeed.MidStrafe;
            }

            // Drives the robot with joysticks from gamepad 1
            Chassis.updateWithControls(
                Gamepad1.getLeftY(), //drive stick
                Gamepad1.getLeftX(), //strafe stick
                Gamepad1.getRightX() //turn stick
            );


            Chassis.addTelemetry(telemetry);


            // Moves the slide with the right joystick on gamepad 2
            // if(gamepad2.left_y != 0) {
            //     Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //     Slide.setPower((gamepad2.left_y));
            // }

            // if (gamepad2.dpad_up) {
            //     Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
            // }

            // if (gamepad2.dpad_down) {
            //     Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Max);
            // }

            // if (gamepad2.dpad_left) {
            //     Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Max);
            // }

            // if (gamepad2.dpad_right) {
            //     Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
            // }

        }
    }
}