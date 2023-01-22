package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorColors;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorDistances;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoSide;

@SuppressWarnings({"ConstantConditions", "CommentedOutCode"})
@Config
@Autonomous(name = "AutoRightPro", group = "A")
public class AutoRightPro extends LinearOpMode {

    public final static int SIDE = AutoSide.Right;

    public static final double finalRot = SIDE == AutoSide.Right ? 0 : 180;
    public static double FastSpeed = 90;
    public static double FastTurnSpeed = 1.6;
    public static double FastAccelSpeed = 75;

    public static double ParkingSpeed = 100;
    public static double ParkingAccelSpeed = 80;

    public static double A_LongDrive = 50; //55
    public static double A_LongAccelSpeed = 80;
    public static double A_LongSpeed = 90;
    public static double A_DetectDist = 17;
    public static double A_DetectTries = 10;
    public static double A_DetectTryMultiplier = 0.25;

    public static double B_FindPoleX = 33;
    public static double B_FindMidPoleY = -13.4;
    public static double B_FindHighPoleY = -9;

    public static double C_PoleAdjust = 0;

    public static double C_LowerAmount = 8;
    public static double C_LowerConeTime = 300;
    public static double C_LowerHighConeTime = 700;

    public static double C_ClearPoleStrafe = 2.5;

    public static double C_ClearHighPoleStrafe = 4;

    public static double D_PickupX = 56;
    public static double D_PickupY = -12;

    public static double D_PickupForward = 8.5;

    public static double E_PickupConeWait = 200;

    public static double E_RotArmDelay = 1;

    public static int ConesToScore = 4;
    public static int ConesOnMid = 2;

    public static double startX = 36;
    public static double startY = -63;

    public static double PoleX = 28;


    /**
     * Ending positions
     */
    public static double EndPos1 = 12;
    public static double EndPos2 = 36;
    public static double EndPos3 = 59;
    private static int ParkingPosition = 2;

    @Override
    public void runOpMode() throws InterruptedException {

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

        Servo podLift = hardwareMap.get(Servo.class, "podLift");
        podLift.setPosition(PodLiftPosition.Down);

        DistanceSensor LeftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor RightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        Pose2d startPose = new Pose2d(SIDE * startX, startY, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

//        telemetry.addData("Autonomous", "AutoRightPro");
//        telemetry.addLine("Amazing");
//        telemetry.update();


        //Timer that will be used to time the autonomous
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        waitForStart();

        if (isStopRequested()) return;

        podLift.setPosition(PodLiftPosition.Down);

        Arm.setRotation(ArmRotation.Center);
        Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Mid); //Sets Slide to low pole height slowly

        final boolean[] GotColor = {false};

//        drive.followTrajectory(
        TrajectoryBuilder FullSpeedDetectionTraj = drive.trajectoryBuilder(startPose)
                .back(A_LongDrive,
                        SampleMecanumDrive.getVelocityConstraint(A_LongSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(A_LongAccelSpeed)
                ); //Drives to high pole

//        drive.setPoseEstimate(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());
//                .addDisplacementMarker(A_DetectDist, () -> { //Reads signal sleeve
//                    SensorColors.Color detectedColor = SensorColors.detectColor(colorSensor);
////                            if(detectedColor != SensorColors.Color.Grey && detectedColor != SensorColors.Color.Unknown) {
////                                GotParkingPos[0] = true;
////                            }
//                    ParkingPosition = SensorColors.getParkingPosition( // reads parking position based of detected color
//                            detectedColor
//                    );
//
//                    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
//
//
//                    telemetry.addData("Detected Color", detectedColor);
////                            dashboard.sendTelemetryPacket(
////                                    new TelemetryPacket().addLine("Detected Color: " + detectedColor)
////                            );
//                    telemetry.addData("Parking Position", ParkingPosition);
//                    telemetry.update();
//                });
//                .build();
//        );


        for (int i = 0; i < A_DetectTries; i++) {
            FullSpeedDetectionTraj.addDisplacementMarker(A_DetectDist + (i * A_DetectTryMultiplier), () -> { //Reads signal sleeve
                Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Max);
                if (GotColor[0] || colorSensor.alpha() < SensorColors.AlphaDetectionMargin) {
                    if (!GotColor[0]) {
//                        telemetry.addData("Got Color", GotColor[0]);
                        telemetry.addData("Alpha", colorSensor.alpha());
                        telemetry.update();
                    }
                    return;
                }

                SensorColors.Color detectedColor = SensorColors.detectColor(colorSensor);
//                            if(detectedColor != SensorColors.Color.Grey && detectedColor != SensorColors.Color.Unknown) {
//                                GotParkingPos[0] = true;
//                            }
                ParkingPosition = SensorColors.getParkingPosition( // reads parking position based of detected color
                        detectedColor
                );

                GotColor[0] = true;


                telemetry.addData("Detected Color", detectedColor);
//                            dashboard.sendTelemetryPacket(
//                                    new TelemetryPacket().addLine("Detected Color: " + detectedColor)
//                            );
                telemetry.addData("Parking Position", ParkingPosition);
                telemetry.update();
            });
        }

        drive.followTrajectory(FullSpeedDetectionTraj.build());


        if (SIDE == AutoSide.Right) {
            Arm.setRotation(ArmRotation.Right);
        } else {
            Arm.setRotation(ArmRotation.Left);
        }//Sets arm to left or right depending on side

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(SIDE * B_FindPoleX, B_FindMidPoleY, Math.toRadians(finalRot)),
                                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, FastTurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        ) //Gets ready to align with high pole
                        .build()
        );

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(SIDE * B_FindPoleX, B_FindMidPoleY, Math.toRadians(finalRot))) //Gets ready to align with high pole
                        .build()
        );

//        drive.followTrajectory(
//                drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .strafeLeft(B_PushConesStrafe)
//                        .build()
//        );
//
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(B_FindPoleX, B_FindPoleY, Math.toRadians(finalRot)),
//                                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, FastTurnSpeed, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        ) //Gets ready to align with high pole
//                        .build()
//        );


        int count = 0;
        while (opModeIsActive() && count < ConesToScore) {
            if (SIDE == AutoSide.Right) {
                if (count < ConesOnMid) {
                    Arm.setRotation(ArmRotation.Right);
                } else {
                    Arm.setRotation(ArmRotation.Left);
                }
            } else {
                if (count < ConesOnMid) {
                    Arm.setRotation(ArmRotation.Left);
                } else {
                    Arm.setRotation(ArmRotation.Right);
                }
            }//Sets arm to left or right depending on side

            Chassis.PoleAlign alignDrive = Chassis.PoleAlign.Backward;
//            double startingAlignTime = timer.seconds();
            int switchAlign = 0;
            boolean foundPole = true;
            while (!drive.autoPlace(Arm, LeftSensor, RightSensor, alignDrive, SIDE == AutoSide.Right ? (count < ConesOnMid ? Chassis.PoleAlign.Right : Chassis.PoleAlign.Left) : (count < ConesOnMid ? Chassis.PoleAlign.Left : Chassis.PoleAlign.Right)) && opModeIsActive()) { //Aligns with high pole
                telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
                telemetry.addData("Right Sensor", RightSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                drive.update();
//                if(startingAlignTime - timer.seconds() > SensorDistances.FindGiveUpTime) {
//                    startingAlignTime = timer.seconds();
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(B_FindPoleX, B_FindPoleY, Math.toRadians(finalRot)),
//                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, FastTurnSpeed, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                                    ) //Gets ready to align with high pole
//                                    .build()
//                    );
//                }
                if (SIDE == AutoSide.Right ? (drive.getPoseEstimate().getX() < (SIDE * PoleX) - SensorDistances.FindBuffer) : (drive.getPoseEstimate().getX() > (SIDE * PoleX) + SensorDistances.FindBuffer)) {
                    switchAlign++;
                    alignDrive = Chassis.PoleAlign.Forward;
                } else if (SIDE == AutoSide.Right ? (drive.getPoseEstimate().getX() > (SIDE * PoleX) + SensorDistances.FindBuffer) : (drive.getPoseEstimate().getX() < (SIDE * PoleX) - SensorDistances.FindBuffer)) {
                    if (switchAlign == 1) switchAlign++;
                    alignDrive = Chassis.PoleAlign.Backward;
                }
                if (switchAlign >= 2) {
                    foundPole = false;
                    break;
                }
            }

            Claw.setOpenAmount(ClawPosition.Open); //Sets claw to open fully

            if (foundPole) {
//                if (C_PoleAdjust > 0) {
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .forward(C_PoleAdjust)
//                                    .build()
//                    ); //Adjusts to be in perfectly lined up with high pole
//                } else if (C_PoleAdjust < 0) {
                if (C_PoleAdjust != 0) {
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .back(C_PoleAdjust)
                                    .build());
                }
//

//            sleep(100);

                Slide.setHeight(count < ConesOnMid ? (SlideHeight.MidPole - C_LowerAmount) : (SlideHeight.HighPole - C_LowerAmount), SlideSpeed.Mid); //Sets Slide to mid pole height slowly

                sleep((long) (count < ConesOnMid ? C_LowerConeTime : C_LowerHighConeTime)); //Lets the slide descend a bit

                Claw.open(); //Opens claw to drop cone
            } else {

                Claw.open();
            }

            Slide.setHeight(count < ConesOnMid ? SlideHeight.MidPole : SlideHeight.HighPole, SlideSpeed.Mid);


//            sleep((long) WaitForConeToDrop);

            if (SIDE == AutoSide.Right) {
                if (count < ConesOnMid) {
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .strafeLeft(count < ConesOnMid ? C_ClearPoleStrafe : C_ClearHighPoleStrafe)
                                    .build()
                    ); //Drives away from high pole to clear it
                } else {
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .strafeRight(count < ConesOnMid ? C_ClearPoleStrafe : C_ClearHighPoleStrafe)
                                    .build()
                    ); //Drives away from high pole to clear it
                }
            } else {
                if (count < ConesOnMid) {
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .strafeRight(count < ConesOnMid ? C_ClearPoleStrafe : C_ClearHighPoleStrafe)
                                    .build()
                    ); //Drives away from high pole to clear it
                } else {
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .strafeLeft(count < ConesOnMid ? C_ClearPoleStrafe : C_ClearHighPoleStrafe)
                                    .build()
                    ); //Drives away from high pole to clear it
                }
            }
//            Claw.open(); //Opens claw to drop cone

            Arm.setRotation(ArmRotation.Center); //Sets arm to center position


//            Claw.close();

//            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
//
//            while (Slide.getInches() < SlideHeight.HighPole - SlideSafetyMargin) {
//                idle();
//            }

            count++; //Increments cone count


//            if ((!BonusCone || ParkingPosition != 3 && timer.seconds() > F_BonusPickupTimeMargin) && count == ConesToScore) break; //If all cones have been scored, break out of loop
            if (count == ConesToScore) break; //If all cones have been scored, break out of loop

//            telemetry.addData("Time Remaining Before Getting Last Cone", 30 - timer.seconds());seconds
//            telemetry.update();

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

            Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - count)), SlideSpeed.Mid); //Sets slide to height of next cone in 5 stack

            Claw.setOpenAmount(ClawPosition.PickupOpen);

            Claw.close();
//            Claw.open();

            int finalCount = count;
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(SIDE * D_PickupX, D_PickupY, Math.toRadians(finalRot)),
                                    SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                            ) //Drives to 5 stack
//                            .addTemporalMarker(0.1, Claw::close) //Closes claw while slide is lowering
                            .addTemporalMarker(0.5, () -> {
                                Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - finalCount)), SlideSpeed.Max); //Sets slide to height of next cone in 5 stack
                            })
                            .build()
            );

//            Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - finalCount)), SlideSpeed.Max); //Sets slide to height of next cone in 5 stack
//            while(Slide.getInches() > SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - count)) + D_PickupSlideWaitMargin) {
//                Claw.open();
//                idle();
//            }

//            Claw.open(); //Opens claw to get ready to pick up cone

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .forward(D_PickupForward,
                                    SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                            ) //Drives forward to pick up cone and align with wall
                            .addDisplacementMarker(0.1, Claw::open)
                            .build()
            );


            //Only reset pose if heading is accurate to D_HeadingMarginToReset of a degree and if D_ResetPoseX is true
//            if (Math.abs(drive.getPoseEstimate().getHeading() - Math.toRadians(finalRot)) < Math.toRadians(D_HeadingMarginToReset) && D_ResetPoseX) {
//              if(E_PickupResetYOffset != 0) drive.setPoseEstimate(new Pose2d(E_PickupResetX, drive.getPoseEstimate().getY() + E_PickupResetYOffset, drive.getPoseEstimate().getHeading())); //Resets X position to be in line with wall
//            }

            Claw.close(); //Closes claw to pick up cone

            sleep((long) E_PickupConeWait); //Waits for cone to be picked up

            Claw.close(); //Closes claw to make sure cone is still picked up

//            if (count == ConesToScore) break; //If all cones have been scored, break out of loop


            if (count < ConesOnMid) Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Max);
            else Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max); //Sets slide to max height

            int finalCount1 = count;
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(SIDE * B_FindPoleX, count < ConesOnMid ? B_FindMidPoleY : B_FindHighPoleY, Math.toRadians(finalRot)),
                                    SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                            ) //Drives to get ready to align with high pole
                            .addTemporalMarker(E_RotArmDelay, () -> {
                                if (SIDE == AutoSide.Right) {
                                    if (finalCount1 < ConesOnMid) {
                                        Arm.setRotation(ArmRotation.Right);
                                    } else {
                                        Arm.setRotation(ArmRotation.Left);
                                    }
                                } else {
                                    if (finalCount1 < ConesOnMid) {
                                        Arm.setRotation(ArmRotation.Left);
                                    } else {
                                        Arm.setRotation(ArmRotation.Right);
                                    }
                                }//Sets arm to left or right depending on side
                            })
                            .build()
            );

        }


        //When all cones have been scored, below executes:
//        if(BonusCone) {
//            telemetry.addData("Time Remaining After Got Cone", 30 - timer.seconds());
////            telemetry.update();
//
////            if(ParkingPosition == 3 || ParkingPosition == 2) {
////                Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Mid); //Sets slide to mid pole height slowly
////
////                drive.followTrajectory(
////                        drive.trajectoryBuilder(drive.getPoseEstimate())
////                                .lineToLinearHeading(new Pose2d(F_FindBonusPoleX, F_FindBonusPoleY, Math.toRadians(finalRot)),
////                                        SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                                        SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
////                                ) //Drives to park
////                                .build()
////                );
////
////                while(Slide.getInches() < SlideHeight.LowPole - D_PickupSlideWaitMargin || Slide.getInches() > SlideHeight.LowPole + D_PickupSlideWaitMargin) {
////                    idle();
////                }
////
////                //Aligns with small pole on right front
////                Chassis.PoleAlign alignDrive = Chassis.PoleAlign.Forward;
////                while (!drive.autoPlace(Arm, LeftSensor, RightSensor, alignDrive, Chassis.PoleAlign.Right) && opModeIsActive()) { //Aligns with high pole
////                    telemetry.addData("Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
////                    telemetry.update();
////                    drive.update();
////                    if (drive.getPoseEstimate().getX() < BonusPoleX - SensorDistances.FindBuffer / 2) {
////                        alignDrive = Chassis.PoleAlign.Forward;
////                    } else if (drive.getPoseEstimate().getX() > BonusPoleX + SensorDistances.FindBuffer) {
////                        alignDrive = Chassis.PoleAlign.Backward;
////                    }
////                }
////
////                drive.followTrajectory(
////                        drive.trajectoryBuilder(drive.getPoseEstimate())
////                                .back(C_PoleAdjust)
////                                .build()
////                ); //Adjusts to be in perfectly lined up with high pole
////
////
////            } else {
////                drive.followTrajectory(
////                        drive.trajectoryBuilder(drive.getPoseEstimate())
////                                .lineToLinearHeading(new Pose2d(EndPos3, D_PickupY, Math.toRadians(finalRot)),
////                                        SampleMecanumDrive.getVelocityConstraint(ParkingSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
////                                        SampleMecanumDrive.getAccelerationConstraint(ParkingAccelSpeed)
////                                ) //Drives to park
////                                .build()
////                );
////            }
//            Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Mid);
//            Claw.close();
//            Arm.setRotation(ArmRotation.Center);
//            switch (ParkingPosition) { //Drives to parking position
//                case 1:
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(EndPos1, D_PickupY, Math.toRadians(finalRot)),
//                                            SampleMecanumDrive.getVelocityConstraint(ParkingSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(ParkingAccelSpeed)
//                                    ) //Drives to end position 1
////                                .addTemporalMarker(1, () -> {
////                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
////                                    Claw.close();
////                                })
//                                    .addDisplacementMarker(5, () -> {
//                                        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Mid); //Sets slide to mid pole height slowly
//                                    })
//                                    .build()
//                    );
//                    break;
//                case 2:
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(EndPos2, D_PickupY, Math.toRadians(finalRot)),
//                                            SampleMecanumDrive.getVelocityConstraint(ParkingSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(ParkingAccelSpeed)
//                                    ) //Drives to end position 2
//                                    .addDisplacementMarker(5, () -> {
//                                        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Mid); //Sets slide to mid pole height slowly
//                                    })
////                                .addTemporalMarker(1, () -> {
////                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
////                                    Claw.close();
////                                })
//                                    .build()
//                    );
//                    break;
//                case 3:
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(EndPos3, D_PickupY, Math.toRadians(finalRot)),
//                                            SampleMecanumDrive.getVelocityConstraint(ParkingSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(ParkingAccelSpeed)
//                                    ) //Drives to end position 3
//                                    .addDisplacementMarker(5, () -> {
//                                        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Mid); //Sets slide to mid pole height slowly
//                                    })
////                                .addTemporalMarker(1, () -> {
////                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
////                                    Claw.close();
////                                })
//                                    .build()
//                    );
//                    break;
//            }
//            Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
//            Claw.close();
//        } else {
        Arm.setRotation(ArmRotation.Center); //Sets arm to center position
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide
        switch (ParkingPosition) { //Drives to parking position
            case 1:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE == AutoSide.Right ? (SIDE * EndPos1) : (SIDE * EndPos3), D_PickupY, Math.toRadians(finalRot)),
                                        SampleMecanumDrive.getVelocityConstraint(ParkingSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(ParkingAccelSpeed)
                                ) //Drives to end position 1
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
                                .lineToLinearHeading(new Pose2d(SIDE * EndPos2, D_PickupY, Math.toRadians(finalRot)),
                                        SampleMecanumDrive.getVelocityConstraint(ParkingSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(ParkingAccelSpeed)
                                ) //Drives to end position 2
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
                                .lineToLinearHeading(new Pose2d(SIDE == AutoSide.Right ? (SIDE * EndPos3) : (SIDE * EndPos1), D_PickupY, Math.toRadians(finalRot)),
                                        SampleMecanumDrive.getVelocityConstraint(ParkingSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(ParkingAccelSpeed)
                                ) //Drives to end position 3
//                                .addTemporalMarker(1, () -> {
//                                    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
//                                    Claw.close();
//                                })
                                .build()
                );
                break;
        }
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide
//            }
        telemetry.addData("Time Remaining", timer.seconds());
        telemetry.update();

        while (opModeIsActive()) { //Waits for autonomous to en
//            telemetry.addData("Time Remaining", timer.seconds());
//            telemetry.addData("Slide", Slide.getInches());
//            telemetry.update();
            idle();
        }
    }
}

