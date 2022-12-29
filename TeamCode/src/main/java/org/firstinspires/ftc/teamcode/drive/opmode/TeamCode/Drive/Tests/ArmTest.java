package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;

@Config
@TeleOp(name = "ArmTest", group = "Tests")
public class ArmTest extends LinearOpMode {

    public static double ArmSpeed = 0.001;

    public static double Delay = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize the arm
        org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));
        Arm.setRotation(ArmRotation.Center);

        // Initialize the gamepad
        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        double ArmTargetPos = ArmRotation.Center;

        waitForStart();

        while (opModeIsActive()) {

//            if(Gamepad2.wasJustPressed(GamepadKeys.Button.X)) {
//                ArmTargetPos = ArmRotation.Left;
//            } else if(Gamepad2.wasJustPressed(GamepadKeys.Button.B)) {
//                ArmTargetPos = ArmRotation.Right;
//            } else if(Gamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
//                ArmTargetPos = ArmRotation.Center;
//            }
//
//
//            if(Arm.getRotation() < ArmTargetPos) {
//                Arm.setRotation(Arm.getRotation() + ArmSpeed);
//            } else if(Arm.getRotation() > ArmTargetPos) {
//                Arm.setRotation(Arm.getRotation() - ArmSpeed);
//            }



            if(Gamepad2.wasJustPressed(GamepadKeys.Button.X)) {
                Arm.setTargetRotation(ArmRotation.Left);
            } else if(Gamepad2.wasJustPressed(GamepadKeys.Button.B)) {
                Arm.setTargetRotation(ArmRotation.Right);
            } else if(Gamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
                Arm.setTargetRotation(ArmRotation.Center);
            }

            if(Gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                telemetry.addData("Arm Speed (Default)", ArmRotation.MaxSpeed);
                Arm.runToTargetRotation(ArmRotation.MaxSpeed);
            } else {
                telemetry.addData("Arm Speed", Arm.getRunSpeed(Arm.getTargetRotation(), Arm.getRotation()));
                Arm.runToTargetRotation();
            }

            telemetry.addData("Arm Rotation", Arm.getRotation());
            telemetry.addData("Arm Target Pos", Arm.getTargetRotation());
            telemetry.update();

//            if(Delay > 0) {
//                sleep((long) Delay);
//            }
            Gamepad2.readButtons();
        }
    }
}
//
////            arm.updateWithControls(
////                    Gamepad2.getRightX(),
////                    gamepad2.x,
////                    gamepad2.b,
////                    gamepad2.y
////            );
//        }
//    }
//}
