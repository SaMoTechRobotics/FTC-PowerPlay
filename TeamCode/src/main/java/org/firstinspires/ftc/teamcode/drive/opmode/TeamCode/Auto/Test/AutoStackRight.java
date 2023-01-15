package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorColors;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorDistances;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;


@Config
@Autonomous(name = "AutoStackRight", group = "AutoTest")
@Disabled
public class AutoStackRight extends LinearOpMode {
    public static int ConesToScore = 5;

    public static double DriveSpeed = 20;
    public static double AccelSpeed = 30;
    public static double TurnSpeed = Math.toRadians(50);

    public static double startX = 55.5; //63
    public static double startY = -12;
    public static double startRot = 0;

    public static double resetX = 63;

    public static double finalRot = 0;

    public static double A_StartForward = 12;

    public static double A_PoleX = 27;
    public static double A_PoleY = -9;

    public static double B_PickupX = 57;
    public static double B_PickupY = -12;

    public static double B_PickupForward = 10;



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

        DistanceSensor LeftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor RightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startRot));

        drive.setPoseEstimate(startPose);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        waitForStart();

        if (isStopRequested()) return;
        int count = 1;

        Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - count)), SlideSpeed.Mid); //Sets slide to height of next cone in 5 stack

        sleep(500);

        Claw.open();

        drive.followTrajectory(
                drive.trajectoryBuilder(startPose)
                        .forward(A_StartForward)
                        .build()
        );

        sleep(500);
        drive.setPoseEstimate(new Pose2d(resetX, startY, startRot));
        sleep(500);

        while(opModeIsActive() && count < ConesToScore) {
            Claw.close();

            sleep(500);

            Arm.setRotation(ArmRotation.Center);
            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

            sleep(500);

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(A_PoleX, A_PoleY, Math.toRadians(finalRot)),
                                    SampleMecanumDrive.getVelocityConstraint(DriveSpeed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                            )
                            .build()
            );

            Arm.setRotation(ArmRotation.Left);

            sleep(1000);

            Claw.open();

            sleep(1000);

            count++;


            Arm.setRotation(ArmRotation.Center);
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(B_PickupX, B_PickupY, Math.toRadians(finalRot)))
                            .build()
            );

            Claw.close();

            if(count >= ConesToScore) break;

            Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - count)), SlideSpeed.Mid); //Sets slide to height of next cone in 5 stack

            while(Slide.getInches() > SlideHeight.Ground + (SlideHeight.StackConeHeight * (5 + 1 - count)) + 10) {
                idle();
            }

            Claw.open();

            sleep(500);

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(B_PickupX, B_PickupY, Math.toRadians(finalRot)))
                            .build()
            );

            drive.followTrajectory(
                    drive.trajectoryBuilder(startPose)
                            .forward(B_PickupForward)
                            .build()
            );

        }

        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
        while(opModeIsActive()) {
            idle();
        }

    }
}