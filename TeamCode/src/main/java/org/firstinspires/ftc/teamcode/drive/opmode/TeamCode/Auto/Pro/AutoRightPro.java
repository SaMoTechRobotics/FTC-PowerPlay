package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.ChassisConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorColors;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorDistances;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;

@Config
@Autonomous(name = "AutoRightPro", group = "AutoPro")
public class AutoRightPro extends LinearOpMode {

    public static double driveToSignalDistance = 18;

    public static double longDriveDistance = 32;

    public static double alignToParkingColumnDistance = 24;

    public static double parkingDistance = 16;

    public static int ConesToScore = 5;

    private static int ParkingPosition = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));
        Arm.setRotation(ArmRotation.Center);

        Claw Claw = new Claw(
                hardwareMap.get(Servo.class, "claw"),
                hardwareMap.get(DistanceSensor.class, "leftDistanceSensor")
        );
        Claw.close();

        DistanceSensor leftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        Pose2d startPose = new Pose2d(-60, -48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Mid);

        colorSensor.enableLed(true);

        drive.followTrajectory(drive
                .trajectoryBuilder(startPose)
                .back(driveToSignalDistance)
                .build()
        ); //drive to cone to read parking position

        sleep(100);

        ParkingPosition = SensorColors.getParkingPosition( // reads parking position based of detected color
                SensorColors.detectColor(colorSensor) //detects color
        );

        telemetry.addData("Detected Color", SensorColors.detectColor(colorSensor));
        telemetry.addData("Parking Position", ParkingPosition);
        telemetry.update();

        sleep(100);

        colorSensor.enableLed(false);

        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(longDriveDistance)
                        .build()
        ); //drives to where it will deliver cones

        drive.turn(Math.toRadians(90)); //turns to face wall

        int count = 0;
        while (opModeIsActive() && count < ConesToScore) {
            drive.alignWithPoleAsync(leftSensor, SensorDistances.DetectAmount, opModeIsActive());

            Arm.setRotation(ArmRotation.Left);

            drive.alignPlaceDistanceAsync(leftSensor, SensorDistances.PlaceDistance, SensorDistances.PlaceMargin, opModeIsActive());

            sleep(500);

            Slide.setHeight(SlideHeight.HighPole - 10, SlideSpeed.Mid);

            sleep(1000);

            Claw.open();

            sleep(1000);

            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Mid);

            sleep(1000);

            drive.alignPlaceDistanceAsync(leftSensor, SensorDistances.CenterDistance, SensorDistances.PlaceMargin, opModeIsActive());

            Arm.setRotation(ArmRotation.Center);
            Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (4 - count)), SlideSpeed.Max);
            Claw.close();

            count++;

            if (count == ConesToScore) {
                Trajectory forwardTraj = drive
                        .trajectoryBuilder(drive.getPoseEstimate())
                        .forward(ChassisConstants.HalfTileWidth)
                        .build();

                drive.followTrajectory(forwardTraj);

                break;
            }
            Trajectory forwardTraj = drive
                    .trajectoryBuilder(drive.getPoseEstimate())
                    .forward(ChassisConstants.HalfTileWidth * 2)
                    .build();

            drive.followTrajectory(forwardTraj);

            sleep(1000);

            Claw.open();

            Trajectory pickupTraj = drive
                    .trajectoryBuilder(forwardTraj.end())
                    .forward(ChassisConstants.HalfTileWidth)
                    .build();

            drive.followTrajectory(pickupTraj);

            Claw.close();

            sleep(1000);

            Claw.close();

            Slide.setHeight(SlideHeight.MaxHeight, SlideSpeed.Max);

            Trajectory backTraj = drive
                    .trajectoryBuilder(pickupTraj.end())
                    .back(ChassisConstants.HalfTileWidth * 2)
                    .build();

            drive.followTrajectory(backTraj);
        }


        switch (ParkingPosition) {
            case 1:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(alignToParkingColumnDistance)
                                .build()
                );
                break;
            case 2:
                break;
            case 3:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(alignToParkingColumnDistance)
                                .build()
                );
                break;
        }
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeRight(parkingDistance)
                        .build()
        );


    }
}
