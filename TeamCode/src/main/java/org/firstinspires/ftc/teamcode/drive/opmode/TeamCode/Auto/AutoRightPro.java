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
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoPositions;

@Config
@Autonomous(name = "AutoRightPro", group = "AutoPro")
public class AutoRightPro extends LinearOpMode {

    public static int ConesToScore = 2;

    public static double driveToSignalDistance = 18;

    public static double endingLongStrafeY = -12;

    public static double strafePosX = 36;
    public static double strafePosY = -13;

    public static double pickUpConeDrive = 6;


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

        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max); //Raises slide to high pole
        Arm.setRotation(ArmRotation.Center); //Sets arm to center position

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(strafePosX, endingLongStrafeY, Math.toRadians(0)))
                        .build()
        ); //Drives to get setup for aligning with pole

        int count = 0; //Counts how many cones have been scored
        while (opModeIsActive() && count < ConesToScore) { //Loops until all target cones have been scored
            Chassis.PoleAlign alignDrive = Chassis.PoleAlign.Backward; //Sets the direction to align with the pole
            while (!drive.autoPlace(Arm, LeftSensor, RightSensor, alignDrive, Chassis.PoleAlign.Left) && opModeIsActive()) { //Loops until the robot is aligned with the pole
                telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                drive.update();
                if (drive.getPoseEstimate().getX() < 24 - SensorDistances.FindBuffer / 2) { //If the robot is too far to the left, reverse the direction to align with the pole
                    alignDrive = Chassis.PoleAlign.Forward;
                } else if (drive.getPoseEstimate().getX() > 24 + SensorDistances.FindBuffer) { //If the robot is too far to the right, reverse the direction to align with the pole
                    alignDrive = Chassis.PoleAlign.Backward;
                }
            }

            sleep(500); //Waits for the robot to settle

            Slide.setHeight(SlideHeight.HighPole - 10, SlideSpeed.Mid); //Lowers slide to ensure cone delivery

            sleep(1000); //Waits for the slide to lower

            Claw.open(); //Opens claw to drop cone

            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Mid); //Raises slide to clear pole

            sleep(500); //Waits for the slide to raise

            count++; //Increments the cone count given that a cone has been scored

            Arm.setRotation(ArmRotation.Center); //Sets arm to center position

            if (count == ConesToScore) break; //Breaks the loop if all target cones have been scored

            int finalCount = count; //Creates a final variable to be used in the lambda function
            drive.followTrajectory( //Drives to the next cone
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(60, strafePosY, Math.toRadians(0)))
                            .addTemporalMarker(0.5, () -> { //Lowers the slide to pick up the next cone
                                Slide.setHeight(SlideHeight.Ground + (SlideHeight.StackConeHeight * (ConesToScore - 1 - finalCount)), SlideSpeed.Max);
                                Claw.close();
                            })
                            .build()
            );

            Claw.open(); //Opens claw to pick up cone

            Trajectory pickupTraj = drive
                    .trajectoryBuilder(drive.getPoseEstimate())
                    .forward(pickUpConeDrive)
                    .build();
            drive.followTrajectory(pickupTraj); //Drives forward to pick up cone

            Claw.close(); //closes claw

            sleep(500); //wait for claw to close

            Claw.close(); //closes claw as backup

            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max); //Raises slide to high pole

            drive.followTrajectory(
                    drive.trajectoryBuilder(pickupTraj.end())
                            .back(30)
                            .build()
            ); //Drives back to deliver cone
        }


        Pose2d ParkingPose = AutoPositions.ParkingRight2; //Sets the parking position to the default position
        switch (ParkingPosition) { //Sets the parking position based on the detected color
            case 1:
                ParkingPose = AutoPositions.ParkingRight1;
                break;
            case 2:
                ParkingPose = AutoPositions.ParkingRight2;
                break;
            case 3:
                ParkingPose = AutoPositions.ParkingRight3;
                break;
        }
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(ParkingPose.getX(), ParkingPose.getY(), Math.toRadians(0)))
                        .addTemporalMarker(1, () -> {
                            Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Lowers slide to park and finish autonomous
                            Claw.close(); //Closes claw to park
                        })
                        .build()
        ); //Drives to the parking position

        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Lowers slide to park and finish autonomous
        Claw.close(); //Closes claw to park

        while (opModeIsActive()) { //Loops until the opmode is stopped
            idle();
        }
    }
}
