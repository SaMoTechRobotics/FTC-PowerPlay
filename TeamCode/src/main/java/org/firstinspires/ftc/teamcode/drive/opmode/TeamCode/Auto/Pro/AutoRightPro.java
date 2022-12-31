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
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name = "AutoRightPro", group = "AutoPro")
public class AutoRightPro extends LinearOpMode {
    public static double FastSpeed = 100;
    public static double FastTurnSpeed = 80;
    public static double FastAccelSpeed = 100;

    public static double A_LongDrive = 55;
    public static double A_DetectDist = 18;

    public static double B_FindPoleX = 28;
    public static double B_FindPoleY = -13;

    public static double C_PoleAdjust = 1;

    public static double C_LowerConeTime = 900;

    public static double C_ClearPoleStrafe = 5;

    public static double D_PickupX = 60;
    public static double D_PickupY = -13;

    public static double D_PickupForward = 10;

    public static double E_PickupConeWait = 500;
    public static double E_PickupResetX = 64;





//    public static double SpeedUpAmount = 20;
//    public static double AccelUpAmount = 20;

//    public static double DropWait = 900;

//    public static double WaitForConeToDrop = 100;

//    public static double driveToSignalDistance = 18;

//    public static double ReadingWait = 100;

//    public static double endingLongStrafeY = -13;

//    public static double strafePosX = 36;
//    public static double strafePosY = -11.5; //-12.5

//    public static double pickUpConeDrive = 7;

    public static int ConesToScore = 3;
    public static double startX = 36;
    public static double startY = -64;

    public static double finalRot = 0;

//    public static double PickupX = 60;

    public static double PoleX = 24;

//    public static double PoleAdjust = 1;

//    public static double SlideSafetyMargin = 0.5;

    /**
     * Ending positions
     */
    public static double EndPos1 = 16;
    public static double EndPos2 = 38;
    public static double EndPos3 = 60;
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

        telemetry.addData("Autonomous", "AutoRightPro");
        telemetry.addLine("Autonomous Overview:");
        telemetry.addLine("1. Drives directly towards signal cone until close to high pole (reads signal sleeve)");
        telemetry.addLine("2. Turns to face stack of 5 then backs up and aligns with high pole");
        telemetry.addLine("3. Drops cone while driving away from high pole");
        telemetry.addLine("4. Drives to stack of 5 cones");
        telemetry.addLine("5. Picks up cone and drives to high pole");
        telemetry.addLine("6. Repeats steps 3-5 until " + ConesToScore + " cones are scored");
        telemetry.addLine("7. Drives to parking position");
        telemetry.addLine("8. Autonomous finishes awesome!");
        telemetry.update();


        //Timer that will be used to time the autonomous
        ElapsedTime timer = new ElapsedTime();
        timer.reset();


        waitForStart();

        if (isStopRequested()) return;

        Arm.setRotation(ArmRotation.Center);
        Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Mid);

//
//        drive.followTrajectory(drive
//                .trajectoryBuilder(startPose)
//                .back(driveToSignalDistance)
//                .build()
//        ); //drive to cone to read parking position
//
////        sleep((long) ReadingWait);
//
//        ParkingPosition = SensorColors.getParkingPosition( // reads parking position based of detected color
//                SensorColors.detectColor(colorSensor) //detects color
//        );
//
//        telemetry.addData("Detected Color", SensorColors.detectColor(colorSensor));
//        telemetry.addData("Parking Position", ParkingPosition);
//        telemetry.update();
//
////        sleep(ReadingWait);
//
////        colorSensor.enableLed(false);
//
//        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
//        Arm.setRotation(ArmRotation.Center);
//
////        drive.followTrajectory(
////                drive.trajectoryBuilder(drive.getPoseEstimate())
////                        .back(longDriveDistance)
////                        .build()
////        ); //drives to where it will deliver cones
//
////        drive.turn(Math.toRadians(90)); //turns to face wall
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(strafePosX, endingLongStrafeY, Math.toRadians(finalRot)),
//                                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + SpeedUpAmount, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + AccelUpAmount)
//                        )
//                        .build()
//        );

        drive.followTrajectory(
                drive.trajectoryBuilder(startPose)
                        .back(A_LongDrive,
                                SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                        )
                        .addDisplacementMarker(A_DetectDist, () -> {
                            SensorColors.Color detectedColor = SensorColors.detectColor(colorSensor);
//                            if(detectedColor != SensorColors.Color.Grey && detectedColor != SensorColors.Color.Unknown) {
//                                GotParkingPos[0] = true;
//                            }
                            ParkingPosition = SensorColors.getParkingPosition( // reads parking position based of detected color
                                    detectedColor
                            );

                            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);


                            telemetry.addData("Detected Color", SensorColors.detectColor(colorSensor));
                            telemetry.addData("Parking Position", ParkingPosition);
                            telemetry.update();
                        })
                        .build()
        );

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(B_FindPoleX, B_FindPoleY, Math.toRadians(finalRot)),
                                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, FastTurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build()
        );

        int count = 0;
        while (opModeIsActive() && count < ConesToScore) {

            Chassis.PoleAlign alignDrive = Chassis.PoleAlign.Backward;
            while (!drive.autoPlace(Arm, LeftSensor, RightSensor, alignDrive, Chassis.PoleAlign.Left) && opModeIsActive()) {
                telemetry.addData("Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                drive.update();
                if (drive.getPoseEstimate().getX() < PoleX - SensorDistances.FindBuffer / 2) {
                    alignDrive = Chassis.PoleAlign.Forward;
                } else if (drive.getPoseEstimate().getX() > PoleX + SensorDistances.FindBuffer) {
                    alignDrive = Chassis.PoleAlign.Backward;
                }
            }

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(C_PoleAdjust)
                            .build()
            );

//            sleep(100);

            Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Mid);

            sleep((long) C_LowerConeTime);

            Claw.open();

//            sleep((long) WaitForConeToDrop);

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeRight(C_ClearPoleStrafe)
                            .build()
            );

            Arm.setRotation(ArmRotation.Center);

//            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
//
//            while (Slide.getInches() < SlideHeight.HighPole - SlideSafetyMargin) {
//                idle();
//            }

            count++;

//            Arm.setRotation(ArmRotation.Center);
            if (count == ConesToScore) break;

//            int finalCount = count;
//            drive.followTrajectory(
//                    drive.trajectoryBuilder(drive.getPoseEstimate())
//                            .lineToLinearHeading(new Pose2d(PickupX, strafePosY, Math.toRadians(finalRot)))
//                            .addTemporalMarker(0.2, () -> {
//                                Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - finalCount)), SlideSpeed.Max);
//                                Claw.close();
//                            })
//                            .build()
//            );

//            Claw.open();
//
//            Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (ConesToScore - 1 - finalCount)), SlideSpeed.Max);
//            Claw.close();
//
//            Trajectory forwardTraj = drive
//                    .trajectoryBuilder(drive.getPoseEstimate())
//                    .forward(ChassisConstants.HalfTileWidth * 2)
//                    .build();

//            drive.followTrajectory(forwardTraj);

            Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - count)), SlideSpeed.Max);

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(D_PickupX, D_PickupY, Math.toRadians(finalRot)),
                                    SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                            )
                            .addTemporalMarker(0.2, Claw::close)
                            .build()
            );

            Claw.open();

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .forward(D_PickupForward)
                            .build()
            );

            drive.setPoseEstimate(new Pose2d(E_PickupResetX, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));

            Claw.close();

            sleep((long) E_PickupConeWait);

            Claw.close();

            Slide.setHeight(SlideHeight.MaxHeight, SlideSpeed.Max);

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(B_FindPoleX, B_FindPoleY, Math.toRadians(finalRot)),
                                    SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                            )
                            .build()
            );

        }

        Arm.setRotation(ArmRotation.Center);
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
        Claw.close();
        switch (ParkingPosition) {
            case 1:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(EndPos1, D_PickupY, Math.toRadians(finalRot)))
//                                .addTemporalMarker(1, () -> {
//                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
//                                    Claw.close();
//                                })
                                .build()
                );
                break;
            case 2:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(EndPos2, D_PickupY, Math.toRadians(finalRot)))
//                                .addTemporalMarker(1, () -> {
//                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
//                                    Claw.close();
//                                })
                                .build()
                );
                break;
            case 3:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(EndPos3, D_PickupY, Math.toRadians(finalRot)))
//                                .addTemporalMarker(1, () -> {
//                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
//                                    Claw.close();
//                                })
                                .build()
                );
                break;
        }
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
        Claw.close();
        while (opModeIsActive()) {
            telemetry.addData("Timer", timer.seconds());
            telemetry.addData("Slide", Slide.getInches());
            telemetry.update();
            idle();
        }
    }
}
