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

    public static double D_HeadingMarginToReset = 0.2;

    public static boolean D_ResetPoseX = true;

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
        Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Mid); //Sets Slide to low pole height slowly

        drive.followTrajectory(
                drive.trajectoryBuilder(startPose)
                        .back(A_LongDrive,
                                SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                        ) //Drives to high pole
                        .addDisplacementMarker(A_DetectDist, () -> { //Reads signal sleeve
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
                        ) //Gets ready to align with high pole
                        .build()
        );

        int count = 0;
        while (opModeIsActive() && count < ConesToScore) {

            Chassis.PoleAlign alignDrive = Chassis.PoleAlign.Backward;
            while (!drive.autoPlace(Arm, LeftSensor, RightSensor, alignDrive, Chassis.PoleAlign.Left) && opModeIsActive()) { //Aligns with high pole
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
            ); //Adjusts to be in perfectly lined up with high pole

//            sleep(100);

            Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Mid); //Sets Slide to mid pole height slowly

            sleep((long) C_LowerConeTime); //Lets the slide descend a bit

            Claw.open(); //Opens claw to drop cone

//            sleep((long) WaitForConeToDrop);

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeRight(C_ClearPoleStrafe)
                            .build()
            ); //Drives away from high pole to clear it

            Arm.setRotation(ArmRotation.Center); //Sets arm to center position

//            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
//
//            while (Slide.getInches() < SlideHeight.HighPole - SlideSafetyMargin) {
//                idle();
//            }

            count++; //Increments cone count

            if (count == ConesToScore) break; //If all cones have been scored, break out of loop

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

            Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - count)), SlideSpeed.Max); //Sets slide to height of next cone in 5 stack

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(D_PickupX, D_PickupY, Math.toRadians(finalRot)),
                                    SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                            ) //Drives to 5 stack
                            .addTemporalMarker(0.2, Claw::close) //Closes claw while slide is lowering
                            .build()
            );

            Claw.open(); //Opens claw to get ready to pick up cone

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .forward(D_PickupForward) //Drives forward to pick up cone and align with wall
                            .build()
            );


            //Only reset pose if heading is accurate to D_HeadingMarginToReset of a degree and if D_ResetPoseX is true
            if (Math.abs(drive.getPoseEstimate().getHeading() - Math.toRadians(finalRot)) < Math.toRadians(D_HeadingMarginToReset) && D_ResetPoseX) {
                drive.setPoseEstimate(new Pose2d(E_PickupResetX, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())); //Resets X position to be in line with wall
            }

            Claw.close(); //Closes claw to pick up cone

            sleep((long) E_PickupConeWait); //Waits for cone to be picked up

            Claw.close(); //Closes claw to make sure cone is still picked up

            Slide.setHeight(SlideHeight.MaxHeight, SlideSpeed.Max); //Sets slide to max height

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(B_FindPoleX, B_FindPoleY, Math.toRadians(finalRot)),
                                    SampleMecanumDrive.getVelocityConstraint(FastSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(FastAccelSpeed)
                            ) //Drives to get ready to align with high pole
                            .build()
            );

        }


        //When all cones have been scored, below executes:

        Arm.setRotation(ArmRotation.Center); //Sets arm to center position
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide
        switch (ParkingPosition) { //Drives to parking position
            case 1:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(EndPos1, D_PickupY, Math.toRadians(finalRot))) //Drives to end position 1
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
                                .lineToLinearHeading(new Pose2d(EndPos2, D_PickupY, Math.toRadians(finalRot))) //Drives to end position 2
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
                                .lineToLinearHeading(new Pose2d(EndPos3, D_PickupY, Math.toRadians(finalRot))) //Drives to end position 3
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

        while (opModeIsActive()) { //Waits for autonomous to end
            telemetry.addData("Timer", timer.seconds());
            telemetry.addData("Slide", Slide.getInches());
            telemetry.update();
            idle();
        }
    }
}
