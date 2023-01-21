package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Fun;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;

@Config
@Autonomous(name = "AutoNavField", group = "Fun")
public class AutoNavField extends LinearOpMode {

    public static double AccelSpeed = 80;
    public static double Speed = 100;
    public static double TurnSpeed = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Servo podLift = hardwareMap.get(Servo.class, "podLift");
        podLift.setPosition(PodLiftPosition.Down);

        Pose2d startPose = new Pose2d(64, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d(64, -63, Math.toRadians(90)))
                        .strafeTo(new Vector2d(60, -60),
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .forward(35,
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .splineToLinearHeading(new Pose2d(48, -12, Math.toRadians(180)), Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .forward(24,
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .splineToLinearHeading(new Pose2d(10, -25, Math.toRadians(270)), Math.toRadians(270),
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .forward(25,
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .splineToLinearHeading(new Pose2d(24, -60, Math.toRadians(0)), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .forward(10,
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .lineToLinearHeading(new Pose2d(50, -60, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(Speed, TurnSpeed, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(AccelSpeed)
                        )
                        .strafeTo(new Vector2d(64, -63))
                        .build()
        );


    }
}
