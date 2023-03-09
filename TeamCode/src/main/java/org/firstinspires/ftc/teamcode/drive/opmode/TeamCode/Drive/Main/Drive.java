package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Main;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;

@Config
@TeleOp(name = "Drive", group = "A")
public class Drive extends LinearOpMode {

    public static boolean AutoDrive = false;
    public static boolean Debug = false;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Init vars -------------------------------------------------------------------------------
         */

        Chassis Chassis = new Chassis(hardwareMap);

        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));
        Slide.resetToZero();

        Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));
        Arm.setRotation(ArmRotation.Center);

        Claw Claw =
                new Claw(
                        hardwareMap.get(Servo.class, "claw"),
                        hardwareMap.get(DistanceSensor.class, "clawDistanceSensor"),
                        hardwareMap.get(Servo.class, "poleBrace")
                );
        Claw.close();

        Servo podLift = hardwareMap.get(Servo.class, "podLift");
        if (AutoDrive) podLift.setPosition(PodLiftPosition.Down);
        else podLift.setPosition(PodLiftPosition.Up);

//        ColorSensor ColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Initialize the gamepads
        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        ToggleButtonReader clawToggleButton = new ToggleButtonReader(
                Gamepad2,
                GamepadKeys.Button.RIGHT_BUMPER
        ); // The button to toggle the claw

        waitForStart();

        Arm.setRotation(ArmRotation.Center);


        /*
         * Main loop -------------------------------------------------------------------------------
         */

        while (opModeIsActive()) {
//            telemetry.addData("Color Sensor", "");
//            telemetry.addData("Red", ColorSensor.red());
//            telemetry.addData("Green", ColorSensor.green());
//            telemetry.addData("Blue", ColorSensor.blue());
//            telemetry.addData("Alpha", ColorSensor.alpha());
//            telemetry.addData("ARGB", ColorSensor.argb());
//            telemetry.addData("Detected Color", SensorColors.detectColor(ColorSensor.red(), ColorSensor.green(), ColorSensor.blue()));

            /*
             * Chassis -----------------------------------------------------------------------------
             */


            // Updates the chassis speed based on gamepad1 bumpers
            Chassis.updateSpeed(
//                    Slide.getInches() < SlideHeight.MidPole &&
                    Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER),
                    Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
            );

            // Drives the robot with joysticks from gamepad 1, normal format
//            Chassis.updateWithControls(
//                    Math.abs(-Gamepad1.getLeftY()) > ChassisSpeed.JoystickYMargin ? -Gamepad1.getLeftY() : 0, //drive stick
//                    Math.abs(-Gamepad1.getLeftX()) > ChassisSpeed.JoystickXMargin ? -Gamepad1.getLeftX() : 0, //strafe stick
//                    Gamepad1.getRightX(), //turn stick
//                    Gamepad1,
//                    Arm,
//                    Claw
//            );
            Chassis.updateWithControls(
                    Gamepad1,
                    AutoDrive
            );
            telemetry.addLine("");
            telemetry.addData("Left Distance", Chassis.getLeftDistance());
            telemetry.addData("Right Distance", Chassis.getRightDistance());
//            if (AutoDrive) {
//                telemetry.addLine("");
//                telemetry.addLine("Auto Chassis Info");
//                telemetry.addData("Current Row", Field.getCurrentRow(Chassis.getPosition()));
//                telemetry.addData("Current Column", Field.getCurrentColumn(Chassis.getPosition()));
//                telemetry.addData("Current Rounded Heading", Chassis.getRoundedHeading());
//            }

            /*
             * Claw --------------------------------------------------------------------------------
             */

            if (Slide.getInches() < SlideHeight.ClawOpenMargin) {
                Claw.setOpenAmount(ClawPosition.PickupOpen);
            } else {
                Claw.setOpenAmount(ClawPosition.Open);
            }

            if (ClawPosition.AutoClose && Slide.getTicks() < ClawPosition.ResetOpenMargin && Claw.detectedCone() && !Gamepad2.getButton(GamepadKeys.Button.A)) {
                Claw.close();
            } else {
                if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) Claw.close();
                else if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) Claw.open();
            }

//            if (Slide.getInches() < SlideHeight.PoleBraceSafetyHeight) Claw.raisePoleBrace();
//            else {
//                if (Gamepad1.wasJustPressed(GamepadKeys.Button.X)) Claw.raisePoleBrace();
//                else if (Gamepad1.wasJustPressed(GamepadKeys.Button.B)) Claw.raisePoleBraceBack();
//                else if (Gamepad1.wasJustPressed(GamepadKeys.Button.Y))
//                    Claw.lowerPoleBrace(Arm.getRotation(), ClawPosition.PoleBraceAlignDirection.Backward);
//                else if (Gamepad1.wasJustPressed(GamepadKeys.Button.A))
//                    Claw.lowerPoleBrace(Arm.getRotation(), ClawPosition.PoleBraceAlignDirection.Forward);
//            }

//            if (Slide.getInches() > SlideHeight.LowPole) {
//                if (Gamepad1.wasJustPressed(GamepadKeys.Button.A)) Claw.enablePoleBrace(true);
//                else if (Gamepad1.wasJustPressed(GamepadKeys.Button.Y)) Claw.enablePoleBrace(false);
//            }

            telemetry.addLine("");
            telemetry.addData("CLAW OPEN", Claw.isOpen() ? "YES!!!!!!" : "NO!!!!!!!");
            telemetry.addLine("");
            telemetry.addData("Claw Sensor Detected Cone", Claw.detectedCone());
            telemetry.addData("Claw Sensor Distance", Claw.getSensorDistance());

            /*
             * Arm ---------------------------------------------------------------------------------
             */

            if (Slide.getInches() > SlideHeight.SafetyHeight) Arm.updateWithControls(
                    Gamepad2.getRightX(),
                    gamepad2.b,
                    gamepad2.x,
                    gamepad2.y,
                    false
//                    Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5
            );
            else Arm.setRotation(ArmRotation.Center);

            telemetry.addLine("");
            telemetry.addData("Arm Position", Arm.getRotation());

            /*
             * Slide -------------------------------------------------------------------------------
             */

            Slide.updateWithControls(
                    Gamepad2.getLeftY(),
                    Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                    Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT),
                    Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                    Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT),
                    Gamepad2.getButton(GamepadKeys.Button.A),
                    Arm,
                    Claw
            );

            Slide.updateSpeed(Gamepad2.getButton(GamepadKeys.Button.A));

            telemetry.addLine("");
            telemetry.addData("Slide Inches", Slide.getInches());
            telemetry.addData("Slide Ticks", Slide.getTicks());
            telemetry.addData("Slide Status", Slide.getStatus());


            /*
             * General -----------------------------------------------------------------------------
             */

            Gamepad1.readButtons();
            Gamepad2.readButtons();
            if (AutoDrive) Chassis.updatePosition();
            if (Debug) telemetry.update();
            else {
                telemetry.clear();
                telemetry.addData("Slide Height (Inches)", Slide.getInches());
                telemetry.addData("Claw Open", Claw.isOpen());
            }
        }
    }
}