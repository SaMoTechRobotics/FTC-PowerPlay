package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoPositions;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoSide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.TargetPole;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.TrajSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.Vec2;

@SuppressWarnings({"ConstantConditions", "InstantiationOfUtilityClass"})
@Config
@Autonomous(name = "AutoRightPro", group = "A")
public class AutoRightPro extends LinearOpMode {

    /*
     * Global Vars ---------------------------------------------------------------------------------
     */
    public final static int SIDE = AutoSide.Right;
    public static final double FINAL_ROT = SIDE == AutoSide.Right ? 0 : 180;

    public static TrajSpeed FAST_SPEED = new TrajSpeed(70, 80, 90);

    /*
     * Signal Sleeve Detection Vars ----------------------------------------------------------------
     */
    private static class DetectionParams {
        public static double StartingDist = 17;
        public static double DetectAttempts = 10;
        public static double DetectAttemptMultiplier = 0.25;
    }

    public static DetectionParams DETECTION_PARAMS = new DetectionParams();

    /*
     * Trajectory Vars -----------------------------------------------------------------------------
     */

    private static class TrajectoryDistances {
        public static double Back1 = 30;
    }

    public static TrajectoryDistances TRAJECTORY_DISTANCES = new TrajectoryDistances();


    /*
     * Position Vars -------------------------------------------------------------------------------
     */

    private static class TrajectoryLocations {


        public static Vec2 FarHighPolePosition = new Vec2(0, -14);
        public static Vec2 CloseHighPolePosition = new Vec2(35, -10);
        public static Vec2 CloseMidPolePosition = new Vec2(35, -14);

    }

    public static TrajectoryLocations TRAJECTORY_LOCATIONS = new TrajectoryLocations();

    /*
     * Parking Vars --------------------------------------------------------------------------------
     */
    private static class ParkingPositions {
        public static double Pos1 = (SIDE == AutoSide.Right ? 12 : 59);
        public static double Pos2 = 36;

        public static double Pos3 = (SIDE == AutoSide.Right ? 59 : 12);

        public static double Y = -12;
    }

    public static ParkingPositions PARKING_POSITIONS = new ParkingPositions();

    public static TrajSpeed PARKING_SPEED = new TrajSpeed(
            80,
            100
    );
    private static int ParkingPosition = 2;

    /*
     * Scoring & Target Vars -----------------------------------------------------------------------
     */

    /**
     * The goal for auto, the preloaded cone is a given so the overall auto will be:
     * 1 + ConesToScore.Cones (3) = 4 total cones
     */
    private static class ConesToScore {
        public static int Cones = 3;
        public static TargetPole Pole1 = TargetPole.CloseMid;
        public static TargetPole Pole2 = TargetPole.FarHigh;
        public static TargetPole Pole3 = TargetPole.CloseHigh;

        //Extra cones: typically don't count unless cones to score is > 3
        public static TargetPole Pole4 = TargetPole.CloseMid;
        public static TargetPole Pole5 = TargetPole.CloseMid;
    }

    public static ConesToScore CONES_TO_SCORE = new ConesToScore();

    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.reset();
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

        ColorSensor ColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        drive.setPoseEstimate(SIDE == AutoSide.Right ? AutoPositions.StartRight : AutoPositions.StartLeft);

        //Timer that will be used to time the autonomous
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (!isStarted()) {
            telemetry.addData("CloseHighPolePos", TrajectoryLocations.CloseHighPolePosition.X);
            telemetry.addData("Second cone to score target", ConesToScore.Pole2);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;


        Arm.setRotation(ArmRotation.Center); //Sets arm to center position
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide
        switch (ParkingPosition) { //Drives to parking position
            case 1:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos1, ParkingPositions.Y, Math.toRadians(FINAL_ROT)),
                                        SampleMecanumDrive.getVelocityConstraint(PARKING_SPEED.Speed, PARKING_SPEED.Turn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(PARKING_SPEED.Accel)
                                )
                                .build()
                );
                break;
            case 2:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos2, ParkingPositions.Y, Math.toRadians(FINAL_ROT)),
                                        SampleMecanumDrive.getVelocityConstraint(PARKING_SPEED.Speed, PARKING_SPEED.Turn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(PARKING_SPEED.Accel)
                                )
                                .build()
                );
                break;
            case 3:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos3, ParkingPositions.Y, Math.toRadians(FINAL_ROT)),
                                        SampleMecanumDrive.getVelocityConstraint(PARKING_SPEED.Speed, PARKING_SPEED.Turn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(PARKING_SPEED.Accel)
                                )
                                .build()
                );
                break;
        }
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide

        telemetry.addData("Time Remaining", 30 - timer.seconds());
        telemetry.update();

        PoseStorage.CurrentPose = drive.getPoseEstimate();

        while (opModeIsActive()) { //Waits for autonomous to end
            idle();
        }
    }
}

