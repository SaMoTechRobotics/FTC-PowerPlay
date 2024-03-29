package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Fun;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Auto.AutoAlignManager;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoSide;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.stream.Collectors;

@Config
@Autonomous(name = "AlignTest", group = "Fun")
public class AlignTest extends LinearOpMode {

    public static int SIDE = AutoSide.Left;

    public static double WaitForDrop = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        DistanceSensor LeftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor RightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));
        Arm.setRotation(ArmRotation.Center);

        Claw Claw = new Claw(
                hardwareMap.get(Servo.class, "claw"),
                hardwareMap.get(DistanceSensor.class, "clawDistanceSensor"),
                hardwareMap.get(Servo.class, "poleBrace")
        );
        Claw.close();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Servo podLift = hardwareMap.get(Servo.class, "podLift");
        podLift.setPosition(PodLiftPosition.Down);

        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        Pose2d startPose = new Pose2d(30, -12, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

        sleep(1000);

        Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Left : ArmRotation.Right);

        AutoAlignManager.smartAlignReset();
        while (!AutoAlignManager.smartAlign(drive, LeftSensor, RightSensor, Chassis.PoleAlign.Backward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Right : Chassis.PoleAlign.Left) && opModeIsActive()) {
            telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
            telemetry.addLine("");
            telemetry.addData("Right Sensor", RightSensor.getDistance(DistanceUnit.INCH));
            telemetry.addLine("");
            telemetry.update();
        }

        AutoAlignManager.getSmartAlignData().distances.forEach(pos -> {
            telemetry.addLine("At (X: " + pos.Position.getX() + ", Y: " + pos.Position.getY() + ", R: " + pos.Position.getHeading() + ") sensor was: " + pos.SensorDistance);
        });

        telemetry.addLine("");
        telemetry.addData("Calculated Position", AutoAlignManager.getCalculatedPosition());

        ArrayList<AutoAlignManager.AlignPos> sortedAlignData = AutoAlignManager.getSmartAlignData().distances.stream() //streams the list of AlignPos
                .sorted(Comparator.comparingDouble(alignPos -> alignPos.SensorDistance)) //compares the sensor distances to sort list
                .collect(Collectors.toCollection(ArrayList::new)); //back to list

        AutoAlignManager.AlignPos bestAlignPos = sortedAlignData.get(
                AutoAlignManager.SmartAlignData.getBestIndex(sortedAlignData) //gets the best align position
        ); //gets the best align position

        telemetry.addData("Best sensor dist", bestAlignPos.SensorDistance);
        telemetry.update();

//        while (opModeIsActive()) {
//            if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) break;
//            idle();
//        }

        sleep((long) WaitForDrop);

        Claw.open();

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeRight(4)
                        .build()
        );

        Arm.setRotation(ArmRotation.Center);

        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);

        Claw.close();

        while (opModeIsActive()) {
            idle();
        }


    }
}
