package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Park;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorColors;

@Config
//@Autonomous(name = "AutoPark", group = "C")
public class AutoPark extends LinearOpMode {

    public static double ParkingSpeed = 60;
    public static double ParkingAccelSpeed = 40;

    public static double A_LongDrive = 30; //55
    public static double A_LongAccelSpeed = 20;
    public static double A_LongSpeed = 55;
    public static double A_DetectDist = 17;
    public static double A_DetectTries = 10;
    public static double A_DetectTryMultiplier = 0.25;

    public static double B_LongDriveBack = 23;

    public static double startX = 36;
    public static double startY = -63;

    public static double finalRot = 0;

    /**
     * Ending positions
     */
    public static double EndPos1 = 14;
    public static double EndPos2 = 36;
    public static double EndPos3 = 58;
    public static double EndY = -36;
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
                hardwareMap.get(DistanceSensor.class, "clawDistanceSensor"),
                hardwareMap.get(Servo.class, "poleBrace")
        );
        Claw.close();


        Servo podLift = hardwareMap.get(Servo.class, "podLift");
        podLift.setPosition(PodLiftPosition.Down);

        DistanceSensor LeftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor RightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        telemetry.addData("Autonomous", "AutoPark");
        telemetry.addLine("Parks in Correct Spot");
        telemetry.update();


        //Timer that will be used to time the autonomous
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        waitForStart();

        if (isStopRequested()) return;

        Arm.setRotation(ArmRotation.Center);

        final boolean[] GotColor = {false};

//        drive.followTrajectory(
        TrajectoryBuilder FullSpeedDetectionTraj = drive.trajectoryBuilder(startPose)
                .back(A_LongDrive,
                        SampleMecanumDrive.getVelocityConstraint(A_LongSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(A_LongAccelSpeed)
                ); //Drives to high pole

        for (int i = 0; i < A_DetectTries; i++) {
            FullSpeedDetectionTraj.addDisplacementMarker(A_DetectDist + (i * A_DetectTryMultiplier), () -> { //Reads signal sleeve
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

        sleep(100);

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(B_LongDriveBack,
                                SampleMecanumDrive.getVelocityConstraint(A_LongSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(A_LongAccelSpeed)
                        )
                        .build()
        );

        {
            switch (ParkingPosition) { //Drives to parking position
                case 1:
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(EndPos1, EndY, Math.toRadians(finalRot)),
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
                                    .lineToLinearHeading(new Pose2d(EndPos2, EndY, Math.toRadians(finalRot)),
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
                                    .lineToLinearHeading(new Pose2d(EndPos3, EndY, Math.toRadians(finalRot)),
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
}


