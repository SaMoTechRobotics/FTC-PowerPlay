package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;

@TeleOp(name = "Lift Pods", group = "Util")
public class LiftPods extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo podLift = hardwareMap.get(Servo.class, "podLift");

        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        telemetry.addLine("This opmode will allow you to raise and lower the odometry pods");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addLine("Press Right Bumper to raise the pods and Left Bumper to lower the pods");
            telemetry.addLine("");

            if (Gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                podLift.setPosition(PodLiftPosition.Up);
            } else if (Gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                podLift.setPosition(PodLiftPosition.Down);
            }

            Gamepad2.readButtons();

            if(podLift.getPosition() == PodLiftPosition.Up) telemetry.addData("Pod Position", "  UP");
            else if(podLift.getPosition() == PodLiftPosition.Down) telemetry.addData("Pod Position", "  DOWN");
            telemetry.update();
        }
    }
}
