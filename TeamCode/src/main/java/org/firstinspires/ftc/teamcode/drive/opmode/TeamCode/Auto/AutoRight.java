package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorColors;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorDistances;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;

@Config
@Autonomous(name = "AutoRight", group = "AutoPro")
public class AutoRight extends LinearOpMode {

    public static double driveToSignalDistance = 18;

    public static double endingLongStrafeY = -12;

    public static double strafePosX = 36;
    public static double strafePosY = -13;

    public static double pickUpConeDrive = 6;

    public static int ConesToScore = 2;

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
                hardwareMap.get(DistanceSensor.class, "clawDistanceSensor")
        );
        Claw.close();

        DistanceSensor LeftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor RightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        Pose2d startPose = new Pose2d(36, -64, Math.toRadians(270));

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
        Arm.setRotation(ArmRotation.Center);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .back(longDriveDistance)
//                        .build()
//        ); //drives to where it will deliver cones

//        drive.turn(Math.toRadians(90)); //turns to face wall

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(strafePosX, endingLongStrafeY, Math.toRadians(0)))
                        .build()
        );
        int count = 0;
        while (opModeIsActive() && count < ConesToScore) {
//            drive.alignWithPoleAsync(leftSensor, SensorDistances.DetectAmount, opModeIsActive());

//            Arm.setRotation(ArmRotation.Left);

//            drive.alignPlaceDistanceAsync(leftSensor, SensorDistances.PlaceDistance, SensorDistances.PlaceMargin, opModeIsActive());

            Chassis.PoleAlign alignDrive = Chassis.PoleAlign.Backward;
            while (!drive.autoPlace(Arm, LeftSensor, RightSensor, alignDrive, Chassis.PoleAlign.Left) && opModeIsActive()) {
                telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                drive.update();
                if (drive.getPoseEstimate().getX() < 24 - SensorDistances.FindBuffer / 2) {
                    alignDrive = Chassis.PoleAlign.Forward;
                } else if (drive.getPoseEstimate().getX() > 24 + SensorDistances.FindBuffer) {
                    alignDrive = Chassis.PoleAlign.Backward;
                }
            }

            sleep(500);

            Slide.setHeight(SlideHeight.HighPole - 10, SlideSpeed.Mid);

            sleep(1000);

            Claw.open();

            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Mid);

            sleep(500);

//            drive.alignPlaceDistanceAsync(leftSensor, SensorDistances.CenterDistance, SensorDistances.PlaceMargin, opModeIsActive());

            count++;

            Arm.setRotation(ArmRotation.Center);
            if (count == ConesToScore) break;

            int finalCount = count;
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(60, strafePosY, Math.toRadians(0)))
                            .addTemporalMarker(0.5, () -> {
                                Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (ConesToScore - 1 - finalCount)), SlideSpeed.Max);
                                Claw.close();
                            })
                            .build()
            );

            Claw.open();
//
//            Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (ConesToScore - 1 - finalCount)), SlideSpeed.Max);
//            Claw.close();
//
//            Trajectory forwardTraj = drive
//                    .trajectoryBuilder(drive.getPoseEstimate())
//                    .forward(ChassisConstants.HalfTileWidth * 2)
//                    .build();

//            drive.followTrajectory(forwardTraj);


            Trajectory pickupTraj = drive
                    .trajectoryBuilder(drive.getPoseEstimate())
                    .forward(pickUpConeDrive)
                    .build();

            drive.followTrajectory(pickupTraj);

            Claw.close();

            sleep(500);

            Claw.close();

            Slide.setHeight(SlideHeight.MaxHeight, SlideSpeed.Max);

            Trajectory backTraj = drive
                    .trajectoryBuilder(pickupTraj.end())
                    .back(30)
                    .build();

            drive.followTrajectory(backTraj);
        }


        switch (ParkingPosition) {
            case 1:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(16, -14, Math.toRadians(0)))
                                .addTemporalMarker(1, () -> {
                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
                                    Claw.close();
                                })
                                .build()
                );
                break;
            case 2:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(38, -14, Math.toRadians(0)))
                                .addTemporalMarker(1, () -> {
                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
                                    Claw.close();
                                })
                                .build()
                );
                break;
            case 3:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(62, -14, Math.toRadians(0)))
                                .addTemporalMarker(1, () -> {
                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
                                    Claw.close();
                                })
                                .build()
                );
                break;
        }
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
        Claw.close();
        while (opModeIsActive()) {
            idle();
        }
    }
}
