package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Main;

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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
@Autonomous(name = "AutoLeft", group = "Auto")
public class AutoLeft extends LinearOpMode {

    public static double SpeedUpAmount = 20;
    public static double AccelUpAmount = 20;

    public static double DropWait = 900;

    public static double WaitForConeToDrop = 100;

    public static double driveToSignalDistance = 18;

    public static double ReadingWait = 100;

    public static double endingLongStrafeY = -13;

    public static double strafePosX = -36;
    public static double strafePosY = -12.5;

    public static double pickUpConeDrive = 6;

    public static int ConesToScore = 3;
    public static double startX = -36;
    public static double startY = -64;

    public static double finalRot = 180;

    public static double PickupX = -60;

    public static double PoleX = 24;

    public static double PoleAdjust = 1;

    public static double SlideSafetyMargin = 0.5;

    /**
     * Ending positions
     */
    public static double EndPos1 = -60.5;
    public static double EndPos2 = -38;
    public static double EndPos3 = -16;
    private static int ParkingPosition = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));
        Slide.resetToZero();

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

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Mid);

        colorSensor.enableLed(true);

        drive.followTrajectory(drive
                        .trajectoryBuilder(startPose)
                        .back(driveToSignalDistance,
                                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + SpeedUpAmount, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + AccelUpAmount))
//                .addDisplacementMarker(() -> {
//
//                })
                        .build()
        ); //drive to cone to read parking position

//        sleep((long) ReadingWait);

        ParkingPosition = SensorColors.getParkingPosition( // reads parking position based of detected color
                SensorColors.detectColor(colorSensor) //detects color
        );

        telemetry.addData("Detected Color", SensorColors.detectColor(colorSensor));
        telemetry.addData("Parking Position", ParkingPosition);
        telemetry.update();

//        sleep(ReadingWait);

//        colorSensor.enableLed(false);

        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
        Arm.setRotation(ArmRotation.Center);

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(strafePosX, endingLongStrafeY, Math.toRadians(finalRot)),
                                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + SpeedUpAmount, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + AccelUpAmount)
                        )
                        .build()
        );
//        drive.followTrajectory(
//                drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(strafePosX, endingLongStrafeY, Math.toRadians(finalRot)))
//                        .build()
//        );
        int count = 0;
        while (opModeIsActive() && count < ConesToScore) {
//            drive.alignWithPoleAsync(leftSensor, SensorDistances.DetectAmount, opModeIsActive());

//            Arm.setRotation(ArmRotation.Left);

//            drive.alignPlaceDistanceAsync(leftSensor, SensorDistances.PlaceDistance, SensorDistances.PlaceMargin, opModeIsActive());

            Chassis.PoleAlign alignDrive = Chassis.PoleAlign.Backward;
            while (!drive.autoPlace(Arm, LeftSensor, RightSensor, alignDrive, Chassis.PoleAlign.Right) && opModeIsActive()) {
                telemetry.addData("Sensor", RightSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                drive.update();
                if (Math.abs(drive.getPoseEstimate().getX()) < PoleX - SensorDistances.FindBuffer / 2) {
                    alignDrive = Chassis.PoleAlign.Forward;
                } else if (Math.abs(drive.getPoseEstimate().getX()) > PoleX + SensorDistances.FindBuffer) {
                    alignDrive = Chassis.PoleAlign.Backward;
                }
            }
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(PoleAdjust)
                            .build()
            );

            sleep(100);

            Slide.setHeight(SlideHeight.HighPole - 10, SlideSpeed.Mid);

            sleep((long) DropWait);

            Claw.open();

            sleep((long) WaitForConeToDrop);

            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

            while (Slide.getInches() < SlideHeight.HighPole - SlideSafetyMargin) {
                idle();
            }

//            sleep(Wait);

//            drive.alignPlaceDistanceAsync(leftSensor, SensorDistances.CenterDistance, SensorDistances.PlaceMargin, opModeIsActive());

            count++;

            Arm.setRotation(ArmRotation.Center);
            if (count == ConesToScore) break;

            int finalCount = count;
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(PickupX, strafePosY, Math.toRadians(finalRot)))
                            .addTemporalMarker(0.2, () -> {
                                Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - finalCount)), SlideSpeed.Max);
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
                                .lineToLinearHeading(new Pose2d(EndPos1, strafePosY, Math.toRadians(finalRot)))
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
                                .lineToLinearHeading(new Pose2d(EndPos2, strafePosY, Math.toRadians(finalRot)))
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
                                .lineToLinearHeading(new Pose2d(EndPos3, strafePosY, Math.toRadians(finalRot)))
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
