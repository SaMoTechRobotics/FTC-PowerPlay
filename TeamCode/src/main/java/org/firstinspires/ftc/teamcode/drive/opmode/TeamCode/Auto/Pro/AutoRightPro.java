package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro;

import static org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight.StackConeHeights;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Auto.AutoAlignManager;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Camera;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoPositions;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoSide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.TargetPole;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.Vec2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@SuppressWarnings("ALL")
@Config
@Autonomous(name = "AutoRightPro", group = "A")
public class AutoRightPro extends LinearOpMode {

    /*
     * Global Vars ---------------------------------------------------------------------------------
     */
    public final static int SIDE = AutoSide.Left;
    public static final double FINAL_ROT = SIDE == AutoSide.Right ? 0 : 180;

    /*
     * Trajector Speed Vars -----------------------------------------------------------------------
     */

    private static class TrajectorySpeeds {
        public static double MaxSpeed = 90;
        public static double FastSpeed = 60;
        public static double NormalSpeed = 55;

        public static double MaxAccel = 60;
        public static double FastAccel = 45;
        public static double NormalAccel = 35;

        public static double MaxTurn = 180;
        public static double FastTurn = 100;
        public static double NormalTurn = 60;
    }

    public static TrajectorySpeeds TRAJECTORY_SPEEDS = new TrajectorySpeeds();

    /*
     * Trajectory Vars -----------------------------------------------------------------------------
     */

    private static class TrajectoryDistances {
        public static double ClearPole = 2.5;

    }

    public static TrajectoryDistances TRAJECTORY_DISTANCES = new TrajectoryDistances();


    /*
     * Position Vars -------------------------------------------------------------------------------
     */

    private static class TrajectoryLocations {

        public static Vec2 FirstDrive = SIDE == AutoSide.Right ? new Vec2(35, -10) : new Vec2(35, -10);

        // Stack pos                                                right                       left
        public static Vec2 StackPos = SIDE == AutoSide.Right ? new Vec2(64, -13) : new Vec2(64, -13);

        public static Vec2 AvoidByStackPos = SIDE == AutoSide.Right ? new Vec2(46, -13) : new Vec2(46, -13);

        public static class PolePos {
            public double X;
            public double Y;
            public double GiveUpX; // If the robot is past this X value, it will give up on finding the pole
            public double FindX; // The robot will start looking for the pole at this X value

            public double Adjust; // The robot will adjust its position by this amount by driving back

            public PolePos(double findX, double x, double giveUpX, double y, double adjust) {
                X = x;
                Y = y;
                GiveUpX = giveUpX;
                FindX = findX;
                Adjust = adjust;
            }
        }

        public static PolePos FarHighPolePos =
                SIDE == AutoSide.Right ?
                        new PolePos(8, 4, 0, -13.5, 0) :
                        new PolePos(8, 4, 0, -14, 0); //left
        public static PolePos CloseHighPolePos =
                SIDE == AutoSide.Right ?
                        new PolePos(31, 28, 20, -13, 0) : //right
                        new PolePos(31, 28, 20, -13, 0); //left

        public static PolePos CloseMidPolePos =
                SIDE == AutoSide.Right ?
                        new PolePos(32, 31, 27, -13.5, 0) :
                        new PolePos(32, 31, 31, -13.5, 0); //left

    }

    public static TrajectoryLocations TRAJECTORY_LOCATIONS = new TrajectoryLocations();


    /*
     * Util and Delays Vars ----------------------------------------------------------------------------
     */
    private double NextSlideHeightForStack = StackConeHeights[4];
    private int HeightIndex = 5;

    private void updateNextSlideHeight() {
        HeightIndex -= 1;
        NextSlideHeightForStack = StackConeHeights[HeightIndex];
    }

    private static class UtilAndDelays {

        public static double GiveUpDelay = 8;
        public static double PoleWait = 0;
        public static double LowerSlideAmount = 8;

        public static double LowerSlideDelay = 400;

        public static double LowerSlideDelayHighPole = 550;

        public static double OpenClawDelay = 10;
        public static double OpenClawPickupDelay = 0;

        public static double CloseClawDelay = 100;
    }

    public static UtilAndDelays UTIL_AND_DELAYS = new UtilAndDelays();


    /*
     * Parking Vars --------------------------------------------------------------------------------
     */
    private static class ParkingPositions {
        public static double Pos1 = (SIDE == AutoSide.Right ? 12 : 59); //Red Signal Sleeve Position
        public static double Pos2 = 36; //Green Signal Sleeve Position

        public static double Pos3 = (SIDE == AutoSide.Right ? 59 : 12); //Blue Signal Sleeve Position

        public static double Y = -13.5;

        public static double Rot = FINAL_ROT;
    }

    public static ParkingPositions PARKING_POSITIONS = new ParkingPositions();

    private static int ParkingPosition = 2;

    /*
     * Scoring & Target Vars -----------------------------------------------------------------------
     */

    /**
     * The goal for auto, the preloaded cone is a given so the overall auto will be:
     * 1 + ConesToScore.Cones (3) = 4 total cones
     */
    private static class ConesToScore {
        public static int Count = 4;

        public static TargetPole Pole0 = TargetPole.CloseHigh;
        public static TargetPole Pole1 = TargetPole.CloseHigh;
        public static TargetPole Pole2 = TargetPole.CloseHigh;
        public static TargetPole Pole3 = TargetPole.CloseHigh;

        //Extra cones: typically don't count unless cones to score is > 3
        public static TargetPole Pole4 = TargetPole.CloseHigh;
        public static TargetPole Pole5 = TargetPole.CloseHigh;

        public static TargetPole[] getPoles() { //Returns an array of the poles to score on
            return new TargetPole[]{
                    Pole0,
                    Pole1,
                    Pole2,
                    Pole3,
                    Pole4,
                    Pole5
            };
        }
    }

    public static ConesToScore CONES_TO_SCORE = new ConesToScore();

    private Pose2d TempPolePos = null;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Init Vars -------------------------------------------------------------------------------
         */

        PoseStorage.reset();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

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

        ColorSensor ColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        Camera Camera = new Camera(hardwareMap);

        drive.setPoseEstimate(SIDE == AutoSide.Right ? AutoPositions.StartRight : AutoPositions.StartLeft);

        Slide.resetToZero();

        TrajectorySequence firstTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true) //reverse splines
                .splineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.FirstDrive.X, -60, Math.toRadians(270)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                ) //clear the wall
                .splineTo(new Vector2d(SIDE * TrajectoryLocations.FirstDrive.X, -46), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.FastTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                ) //drive around ground junction
                .splineTo(new Vector2d(SIDE * TrajectoryLocations.FirstDrive.X, TrajectoryLocations.FirstDrive.Y), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.FastTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                ) //drive to first high pole and turn
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.CloseHighPolePos.FindX, TrajectoryLocations.CloseHighPolePos.Y, Math.toRadians(FINAL_ROT)),
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.FastTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                ) //drive to first high pole
                .build();

        //Timer that will be used to time the autonomous
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        /*
         * Init Actions -----------------------------------------------------------------------------
         */

        while (!isStarted() && !isStopRequested()) {
            Camera.scanForTags();
            telemetry.addData("Detected", "Tag " + Camera.getDetectedTag());
            telemetry.addLine("");
            telemetry.addData("Poles", Arrays.toString(ConesToScore.getPoles()));
            telemetry.update();
        }

        waitForStart();

        Claw.close();

        if (isStopRequested()) return;

        ParkingPosition = Camera.getDetectedTag();

        Claw.close();
        Arm.setRotation(ArmRotation.Center);
        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

        drive.followTrajectorySequence( //drive away from wall and to high pole
                firstTraj
        );

//        {
//            TrajectoryLocations.PolePos TargetPolePos = TrajectoryLocations.CloseHighPolePos;
//
//            drive.followTrajectory(
//                    drive.trajectoryBuilder(drive.getPoseEstimate())
//                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
//                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
//                            )
//                            .build()
//            );
//        }

        /*
         * Scoring 5 stack -------------------------------------------------------------------------
         */
        int ConesScored = 0;
        TargetPole[] TargetPoles;
        scoreCones:
        while (ConesScored < ConesToScore.Count) {
            TargetPoles = ConesToScore.getPoles();

            /*
             * Score Cone --------------------------------------------------------------------------
             */
            switch (TargetPoles[ConesScored]) {

                /*
                 * Score on far high pole ----------------------------------------------------------
                 */
                case CloseHigh: { //Scope for this case
                    TrajectoryLocations.PolePos TargetPolePos = TrajectoryLocations.CloseHighPolePos;
                    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
                    Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Left : ArmRotation.Right);

                    boolean FoundPole = true;
                    ElapsedTime findTimer = new ElapsedTime();
//                    while (!drive.autoPlace(LeftSensor, RightSensor, Chassis.PoleAlign.Backward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Left : Chassis.PoleAlign.Right) && opModeIsActive()) {
//                        telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
//                        telemetry.addLine("");
//                        telemetry.addData("Right Sensor", RightSensor.getDistance(DistanceUnit.INCH));
//                        telemetry.update();
//
//                        if (
//                                (findTimer.seconds() > UtilAndDelays.GiveUpDelay)
//                                        || (SIDE == AutoSide.Right ?
//                                        (drive.getPoseEstimate().getX() < SIDE * TargetPolePos.GiveUpX) :
//                                        (drive.getPoseEstimate().getX() > SIDE * TargetPolePos.GiveUpX))
//                        ) { //if took too long or went too far, give up
//                            drive.followTrajectory(
//                                    drive.trajectoryBuilder(drive.getPoseEstimate())
//                                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
//                                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
//                                            ) //drive to pole estimate location
//                                            .build()
//                            );
//                            FoundPole = false;
//                            break;
//                        }
//                    }

                    if (TempPolePos != null) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(TempPolePos.getX(), TempPolePos.getY(), TempPolePos.getHeading()),
                                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                        ) //drive to pole estimate location from last time
                                        .build()
                        );
                        TempPolePos = null;
                    } else {
                        AutoAlignManager.smartAlignReset();
                        while (!AutoAlignManager.smartAlign(drive, LeftSensor, RightSensor, Chassis.PoleAlign.Backward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Left : Chassis.PoleAlign.Right) && opModeIsActive()) {
                            telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
                            telemetry.addLine("");
                            telemetry.addData("Right Sensor", RightSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                            if (
                                    (findTimer.seconds() > UtilAndDelays.GiveUpDelay)
                                            || (SIDE == AutoSide.Right ?
                                            (drive.getPoseEstimate().getX() < SIDE * TargetPolePos.GiveUpX) :
                                            (drive.getPoseEstimate().getX() > SIDE * TargetPolePos.GiveUpX))
                            ) { //if took too long or went too far, give up
                                drive.followTrajectory(
                                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                                ) //drive to pole estimate location
                                                .build()
                                );
                                FoundPole = false;
                                break;
                            }
                        }
                        TempPolePos = AutoAlignManager.getCalculatedPosition();
                    }

                    if (UtilAndDelays.PoleWait > 0) sleep((long) UtilAndDelays.PoleWait);

                    Slide.setHeight(SlideHeight.HighPole - UtilAndDelays.LowerSlideAmount, SlideSpeed.Max);

                    sleep((long) UtilAndDelays.LowerSlideDelayHighPole);




                    /*
                     * Driving back to 5 stack -----------------------------------------------------------------
                     */

                    ConesScored++; // Increment cones scored

                    if (ConesScored >= ConesToScore.Count) {
                        Claw.setOpenAmount(ClawPosition.Open);
                        Claw.open();

                        sleep((long) UtilAndDelays.OpenClawDelay);

                        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Min);

                        if (SIDE == AutoSide.Right) {
                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .strafeRight(TrajectoryDistances.ClearPole)
                                            .build()
                            );
                        } else {
                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .strafeLeft(TrajectoryDistances.ClearPole)
                                            .build()
                            );
                        }
                        break scoreCones; // If all cones have been scored, break out of loop
                    }

                    updateNextSlideHeight(); // Update the next slide height
                    Claw.setOpenAmount(ClawPosition.PickupOpen);

                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .addDisplacementMarker(TrajectoryDistances.ClearPole / 4, () -> {
                                        Claw.setOpenAmount(ClawPosition.Open);
                                        Claw.open();

//                                        sleep((long) UtilAndDelays.OpenClawDelay);

                                        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Min);
                                    })
                                    .splineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y - TrajectoryDistances.ClearPole, Math.toRadians(FINAL_ROT)), Math.toRadians(FINAL_ROT),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .addDisplacementMarker(() -> {
                                        Arm.setRotation(ArmRotation.Center);
                                        Slide.setHeight(NextSlideHeightForStack, SlideSpeed.Max);
                                        Claw.close();
                                    })
                                    .addDisplacementMarker(5, () -> {
                                        Claw.open();
                                    })
                                    .splineTo(new Vector2d(SIDE * TrajectoryLocations.AvoidByStackPos.X, TrajectoryLocations.AvoidByStackPos.Y), Math.toRadians(FINAL_ROT),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .addDisplacementMarker(() -> {
                                        Claw.open();
                                    })
                                    .splineTo(new Vector2d(SIDE * TrajectoryLocations.StackPos.X, TrajectoryLocations.StackPos.Y), Math.toRadians(FINAL_ROT),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .build()
                    );
                } //End of scope for this case
                break; //end of delivering to far high pole


                /*
                 * Score on close mid pole ---------------------------------------------------------
                 */
                case CloseMid: {
                } //End of scope for this case
                break; //end of delivering to far high pole

                /*
                 * Score on close high pole ---------------------------------------------------------
                 */
                case FarHigh: {
                } //End of scope for this case
                break; //end of delivering to close high pole


            } //End of scoring cone on pole


//            /*
//             * Picking up cone
//             */
//            Claw.setOpenAmount(ClawPosition.PickupOpen);
//            Claw.open();
//
//            sleep((long) UtilAndDelays.OpenClawPickupDelay);

            Claw.close();
            sleep((long) UtilAndDelays.CloseClawDelay);

            /*
             * Driving to next pole
             */
            switch (TargetPoles[ConesScored]) {
                case CloseHigh: {
                    TrajectoryLocations.PolePos TargetPolePos = TrajectoryLocations.CloseHighPolePos;
                    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max); //set slide height to high pole
//                    drive.followTrajectorySequence(
//                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                    .setReversed(true)
//                                    .splineTo(new Vector2d(SIDE * TrajectoryLocations.AvoidByStackPos.X, TrajectoryLocations.AvoidByStackPos.Y), Math.toRadians(FINAL_ROT + 180),
//                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
//                                    )
//                                    .addDisplacementMarker(() -> {
//                                        Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Left : ArmRotation.Right); //rotate arm to pole
//                                    })
//                                    .splineTo(new Vector2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y), Math.toRadians(FINAL_ROT + 180),
//                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
//                                    ) //drive to stack
//                                    .setReversed(false)
//                                    .build()
//                    );
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .build()
                    );
                } //end of close high
                break;
            } //end of drive to pole case


        } //end of loop

        /*
         * Parking ---------------------------------------------------------------------------------
         */

        Arm.setRotation(ArmRotation.Center); //Sets arm to center position
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide
        switch (ParkingPosition) { //Drives to parking position
            case 1:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos1, ParkingPositions.Y, Math.toRadians(ParkingPositions.Rot)),
                                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.MaxSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.MaxAccel)
                                )
                                .build()
                );
                break;
            case 2:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos2, ParkingPositions.Y, Math.toRadians(ParkingPositions.Rot)),
                                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.MaxSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.MaxAccel)
                                )
                                .build()
                );
                break;
            case 3:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos3, ParkingPositions.Y, Math.toRadians(ParkingPositions.Rot)),
                                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.MaxSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.MaxAccel)
                                )
                                .build()
                );
                break;
        }
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide

        /*
         * End of auto -----------------------------------------------------------------------------
         */

        telemetry.addData("Time Remaining", 30 - timer.seconds());
        telemetry.update();

        PoseStorage.CurrentPose = drive.getPoseEstimate();

        while (opModeIsActive()) { //Waits for autonomous to end
            idle();
        }
    }
}

