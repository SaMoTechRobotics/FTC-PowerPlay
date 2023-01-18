package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Main;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Eyes;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Field.Field;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.ChassisSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Eyes.EyesPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorColors;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

//@Config
@TeleOp(name = "Drive", group = "Drive")
public class Drive extends LinearOpMode {


//    public static double MoveMultiply = 0.5;

//    public static boolean BrakeOn = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Chassis Chassis = new Chassis(hardwareMap);

        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));
        Arm.setRotation(ArmRotation.Center);

        Claw Claw =
                new Claw(
                        hardwareMap.get(Servo.class, "claw"),
                        hardwareMap.get(DistanceSensor.class, "clawDistanceSensor")
                );
        Claw.close();

//        Eyes Eyes = new Eyes(hardwareMap.get(Servo.class, "eyesServo"));

        Servo podLift = hardwareMap.get(Servo.class, "podLift");

        ColorSensor ColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Initialize the gamepad
        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

//        ToggleButtonReader chassisBrakeToggle = new ToggleButtonReader(
//                Gamepad1,
//                GamepadKeys.Button.Y
//        ); // The button that toggles the chassis brake

        ToggleButtonReader clawToggleButton = new ToggleButtonReader(
                Gamepad2,
                GamepadKeys.Button.RIGHT_BUMPER
        ); // The button to toggle the claw

//        telemetry.addData("Reset Slide to Zero Position", "Press A");
//
//        boolean ResetSlide = false;
//
//        while (!isStarted()) {
//            if (Gamepad2.wasJustPressed(GamepadKeys.Button.A)) {
//                ResetSlide = !ResetSlide;
//            }
//            telemetry.addData("Current Slide Ticks", Slide.getTicks());
//            telemetry.addLine("Press A to reset Slide to zero position");
//            telemetry.addData("Slide will reset", ResetSlide);
//            telemetry.update();
//            Gamepad2.readButtons();
//        }

        Slide.resetToZero();

        waitForStart();

        Arm.setRotation(ArmRotation.Center);

//        if (ResetSlide) Slide.resetToZero();

        ColorSensor.enableLed(true);

        double avgStickY = 0;

        while (opModeIsActive()) {
            podLift.setPosition(PodLiftPosition.Up);

            telemetry.addData("Color Sensor", "");
            telemetry.addData("Red", ColorSensor.red());
            telemetry.addData("Green", ColorSensor.green());
            telemetry.addData("Blue", ColorSensor.blue());
            telemetry.addData("Alpha", ColorSensor.alpha());
            telemetry.addData("ARGB", ColorSensor.argb());
            telemetry.addData("Detected Color", SensorColors.detectColor(ColorSensor.red(), ColorSensor.green(), ColorSensor.blue()));


            /* CHASSIS */

            // Toggles the chassis brakes
//            Chassis.toggleBrake(chassisBrakeToggle.getState());
//            chassisBrakeToggle.readValue();

            // Updates the chassis speed based on gamepad1 bumpers
            Chassis.updateSpeed(
//                    Slide.getInches() < SlideHeight.MidPole &&
                    Gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER),
                    Gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
            );

            //enable rotation if right trigger is down
//            boolean rotationEnabled = Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;

            // Drives the robot with joysticks from gamepad 1, format of each joystick being one direction
//            Chassis.updateWithControls(
//                    -Gamepad1.getLeftY(), //drive stick
//                    !rotationEnabled ? -Gamepad1.getRightX() : 0, //strafe stick
//                    rotationEnabled ? Gamepad1.getRightX() : 0 //turn stick
//            );

//            boolean driveAlign = Gamepad1.getButton(GamepadKeys.Button.DPAD_UP);
//            if (Gamepad1.getButton(GamepadKeys.Button.DPAD_UP)) driveAlign = false;
//
//            boolean strafeAlign = Gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT);
//            if (Gamepad1.getButton(GamepadKeys.Button.DPAD_LEFT)) strafeAlign = false;


//            if(Math.abs(Gamepad1.getLeftY()) < 0.1) {
//                if(avgStickY < 0) avgStickY += MoveMultiply;
//                if(avgStickY > 0) avgStickY -= MoveMultiply;
//                if(avgStickY < 0.5 && avgStickY > -0.5) avgStickY = 0;
//            } else {
//                if(Math.abs(avgStickY) < 1) avgStickY += -Gamepad1.getLeftY() * MoveMultiply;
//            }

//            Chassis.toggleBrake(BrakeOn);

            // Drives the robot with joysticks from gamepad 1, normal format
            Chassis.updateWithControls(
                    Math.abs(-Gamepad1.getLeftY()) > ChassisSpeed.JoystickYMargin ? -Gamepad1.getLeftY() : 0, //drive stick
//                    avgStickY,
                    Math.abs(-Gamepad1.getLeftX()) > ChassisSpeed.JoystickXMargin ? -Gamepad1.getLeftX() : 0, //strafe stick
                    Gamepad1.getRightX(), //turn stick
                    Gamepad1,
                    Arm,
                    Claw
            );
            telemetry.addLine("");
            telemetry.addData("Left Distance", Chassis.getLeftDistance());
            telemetry.addData("Right Distance", Chassis.getRightDistance());
            telemetry.addData("Current Row", Field.getCurrentRow(Chassis.getPosition()));
            telemetry.addData("Current Column", Field.getCurrentColumn(Chassis.getPosition()));

            // Pose2d ChassisPos = Chassis.getPosition();
            // telemetry.addData("","");
            // telemetry.addData("Chassis X", ChassisPos.getX());
            // telemetry.addData("Chassis Y", ChassisPos.getY());
            // telemetry.addData("Chassis Heading", ChassisPos.getHeading());

            /* CLAW */

            if(Slide.getInches() < SlideHeight.ClawOpenMargin) {
                Claw.setOpenAmount(ClawPosition.PickupOpen);
            } else {
                Claw.setOpenAmount(ClawPosition.Open);
            }

            if (ClawPosition.AutoClose && Slide.getTicks() < ClawPosition.ResetOpenMargin && Claw.detectedCone() && !Gamepad2.getButton(GamepadKeys.Button.A)) {
                Claw.close();
            } else {
                if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) Claw.close();
                else if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) Claw.open();
//                ) Claw.toggleOpen(clawToggleButton.getState());
            }
//            if (ClawPosition.AutoClose && Slide.getTicks() < SlideHeight.GroundMargin && Claw.detectedCone())
//                Claw.close();

            telemetry.addLine("");
            telemetry.addData("CLAW OPEN", Claw.isOpen() ? "YES!!!!!!" : "NO!!!!!!!");
            telemetry.addLine("");
            telemetry.addData("Claw Position", Claw.getPosition());
            telemetry.addData("Claw Sensor Detected Cone", Claw.detectedCone());
            telemetry.addData("Claw Sensor Distance", Claw.getSensorDistance());

            /* ARM */

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

            /* DRAW ARM AND CLAW */

            TelemetryPacket telemetryTest = new TelemetryPacket();
            DashboardUtil.drawArm(
                    telemetryTest.fieldOverlay(),
                    Chassis.getPosition(),
                    Arm.getRotation(),
                    Claw.isOpen()
            );
//            FtcDashboard.getInstance().sendTelemetryPacket(telemetryTest);

            /* SLIDE */

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

//            Slide.waitForArm(Arm.getRotation());

            Slide.updateSpeed(Gamepad2.getButton(GamepadKeys.Button.A));

            telemetry.addLine("");
            telemetry.addData("Slide Inches", Slide.getInches());
            telemetry.addData("Slide Ticks", Slide.getTicks());
            telemetry.addData("Slide Status", Slide.getStatus());

            /* GENERAL */

            // if (Slide.getInches() < SlideHeight.SafetyMargin) { //make sure this only happens once
            //   Arm.setRotation(ArmRotation.Center);
            //   // if(Slide.getInches() > SlideHeight.SafetyHeight) Claw.close();
            // }
//            if (Slide.getInches() < SlideHeight.SafetyHeight && Arm.getRotation() != ArmRotation.Center) {
//                Slide.setPower(0);
//            } else {
//                Slide.setPower(Slide.Speed);
//            }

            // if(Slide.getInches() < SlideHeight.SafetyHeight && previousSlideHeight >= SlideHeight.SafetyHeight) {
            //   Arm.setRotation(ArmRotation.Center);
            //   // Claw.close();
            // }

//            if (Slide.getInches() < SlideHeight.SafetyMargin) {
//                Arm.setRotation(ArmRotation.Center);
//            }
//            if (
//                    Slide.getInches() < SlideHeight.SafetyHeight &&
//                            Arm.getRotation() != ArmRotation.Center
//            ) {
//                Slide.pause();
//            } else {
//                if (Slide.isPaused) Slide.resume();
//            }

            /* EYES */

//            if (Gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
//                Eyes.setPosition(EyesPosition.Forward);
//            } else if (Slide.getTicks() > SlideHeight.GroundMargin) {
//                Eyes.syncWithSlide(Slide.getTicks());
//            }


            /* TELEMETRY */

            Gamepad1.readButtons();
            Gamepad2.readButtons();
            Chassis.updatePosition();
            telemetry.update();
        }
    }
}