package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.ChassisConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorDistances;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;

@Config
@Autonomous(name = "AutoRightPro", group = "AutoPro")
public class AutoRightPro extends LinearOpMode {

    public static double strafe1 = 54.0;
    public static double back2 = 4.0;

    public static int ConesToScore = 5;

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

        Pose2d startPose = new Pose2d(-60, -48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

        Trajectory strafeLeftTraj = drive
                .trajectoryBuilder(startPose)
                .strafeLeft(strafe1)
                .build();

        drive.followTrajectory(strafeLeftTraj);

//        sleep(2000);

//        Trajectory backTraj = drive
//                .trajectoryBuilder(strafeLeftTraj.end())
//                .back(back2)
//                .build();

//        drive.followTrajectory(backTraj);
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


    }
}
