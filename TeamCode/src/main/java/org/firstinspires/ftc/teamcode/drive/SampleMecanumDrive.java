package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.STOP_DELAY;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.ChassisSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorDistances;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunnerCancelable;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

/*
 * Trajectory-cancelable version of the simple mecanum drive hardware implementation for REV hardware.
 * Ensure that this is copied into your project.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(
            MAX_VEL,
            MAX_ANG_VEL,
            TRACK_WIDTH
    );
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(
            MAX_ACCEL
    );
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(
            4,
            0,
            0
    ); //kP = 4
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(2, 0, 0); //kP = 2
    public static double LATERAL_MULTIPLIER = 1.148005162241888; //1.148005162241888
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    private final TrajectorySequenceRunnerCancelable trajectorySequenceRunner;
    private final TrajectoryFollower follower;

    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;
    private final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;

    private final BNO055IMU imu;
    private final VoltageSensor batteryVoltageSensor;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        follower =
                new HolonomicPIDVAFollower(
                        TRANSLATIONAL_PID,
                        TRANSLATIONAL_PID,
                        HEADING_PID,
                        new Pose2d(0.5, 0.5, Math.toRadians(5.0)),
                        STOP_DELAY
                );

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_X);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor
                    .getMotorType()
                    .clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));

        trajectorySequenceRunner =
                new TrajectorySequenceRunnerCancelable(follower, HEADING_PID);
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(
            double maxVel,
            double maxAngularVel,
            double trackWidth
    ) {
        return new MinVelocityConstraint(
                Arrays.asList(
                        new AngularVelocityConstraint(maxAngularVel),
                        new MecanumVelocityConstraint(maxVel, trackWidth)
                )
        );
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(
            double maxAccel
    ) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(
            Pose2d startPose,
            boolean reversed
    ) {
        return new TrajectoryBuilder(
                startPose,
                reversed,
                VEL_CONSTRAINT,
                ACCEL_CONSTRAINT
        );
    }

    public TrajectoryBuilder trajectoryBuilder(
            Pose2d startPose,
            double startHeading
    ) {
        return new TrajectoryBuilder(
                startPose,
                startHeading,
                VEL_CONSTRAINT,
                ACCEL_CONSTRAINT
        );
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT,
                ACCEL_CONSTRAINT,
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate()).turn(angle).build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(
            TrajectorySequence trajectorySequence
    ) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public void breakFollowing() {
        trajectorySequenceRunner.breakFollowing();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(
                getPoseEstimate(),
                getPoseVelocity()
        );
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(
            DcMotor.ZeroPowerBehavior zeroPowerBehavior
    ) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(
            DcMotor.RunMode runMode,
            PIDFCoefficients coefficients
    ) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p,
                coefficients.i,
                coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (
                Math.abs(drivePower.getX()) +
                        Math.abs(drivePower.getY()) +
                        Math.abs(drivePower.getHeading()) >
                        1
        ) {
            // re-normalize the powers according to the weights
            double denom =
                    VX_WEIGHT *
                            Math.abs(drivePower.getX()) +
                            VY_WEIGHT *
                                    Math.abs(drivePower.getY()) +
                            OMEGA_WEIGHT *
                                    Math.abs(drivePower.getHeading());

            vel =
                    new Pose2d(
                            VX_WEIGHT * drivePower.getX(),
                            VY_WEIGHT * drivePower.getY(),
                            OMEGA_WEIGHT * drivePower.getHeading()
                    )
                            .div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
        // expected). This bug does NOT affect orientation.
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }

    //
//    public void autoAlign(DistanceSensor sensor, double speed, double detect, double place, boolean active) {
//        while (sensor.getDistance(DistanceUnit.INCH) > detect && active) {
//            this.setWeightedDrivePower(
//                    new Pose2d(
//                            -speed,
//                            0,
//                            0
//                    )
//            );
//            this.update();
//        }
//    }
//
//    public void alignWithPoleAsync(DistanceSensor sensor, double align, boolean active) {
//        while (sensor.getDistance(DistanceUnit.INCH) > align && active) {
//            this.setWeightedDrivePower(
//                    new Pose2d(
//                            -ChassisSpeed.AlignSpeed,
//                            0,
//                            0
//                    )
//            );
//            this.update();
//        }
//        this.setWeightedDrivePower(
//                new Pose2d(
//                        0,
//                        0,
//                        0
//                )
//        );
//    }
//
//    public void alignPlaceDistanceAsync(DistanceSensor sensor, double place, double margin, boolean active) {
//        while ((sensor.getDistance(DistanceUnit.INCH) > place + margin || sensor.getDistance(DistanceUnit.INCH) < place - margin) && active) {
//            if (sensor.getDistance(DistanceUnit.INCH) > SensorDistances.DetectAmount)
//                this.alignWithPoleAsync(sensor, );
//            this.setWeightedDrivePower(
//                    new Pose2d(
//                            0,
//                            sensor.getDistance(DistanceUnit.INCH) > place ? ChassisSpeed.PlaceSpeed : -ChassisSpeed.PlaceSpeed,
//                            0
//                    )
//            );
//            this.update();
//        }
//        this.setWeightedDrivePower(
//                new Pose2d(
//                        0,
//                        0,
//                        0
//                )
//        );
//    }
//
//    /**
//     * Pivots around point based on radius
//     *
//     * @param speed  speed of pivot
//     * @param radius radius of pivot
//     */
//    public void pivot(double speed, double radius) {
//        this.setWeightedDrivePower(
//                new Pose2d( //sets the power to drive forward and turn
//                        speed,
//                        0,
//                        speed / radius
//                )
//        );
//    }
    public final boolean autoPlace(DistanceSensor leftSensor, DistanceSensor rightSensor, Chassis.PoleAlign alignDrive, Chassis.PoleAlign alignStrafe) {
        double sensorDistance = alignStrafe == Chassis.PoleAlign.Left ? leftSensor.getDistance(DistanceUnit.INCH) : rightSensor.getDistance(DistanceUnit.INCH);
        double PlaceDistance = alignStrafe == Chassis.PoleAlign.Left ? SensorDistances.LeftPlaceDistance : SensorDistances.RightPlaceDistance;
        if (sensorDistance > SensorDistances.DetectAmount) {
            this.setWeightedDrivePower(
                    new Pose2d(
                            alignDrive == Chassis.PoleAlign.Forward ? ChassisSpeed.AlignSpeed : -ChassisSpeed.AlignSpeed,
                            0,
                            0
                    )
            );
            return false;
        } else if (sensorDistance > PlaceDistance + SensorDistances.PlaceMargin || sensorDistance < SensorDistances.LeftPlaceDistance - SensorDistances.PlaceMargin) {
            this.followTrajectory(
                    this.trajectoryBuilder(this.getPoseEstimate())
                            .back(SensorDistances.DriveBackAdjust)
                            .build()
            );
            double newSensorDistance = alignStrafe == Chassis.PoleAlign.Left ? leftSensor.getDistance(DistanceUnit.INCH) : rightSensor.getDistance(DistanceUnit.INCH);
            if (newSensorDistance < sensorDistance) sensorDistance = newSensorDistance;

            if (sensorDistance > PlaceDistance && sensorDistance - PlaceDistance > 0.01) {
                if (alignStrafe == Chassis.PoleAlign.Left) {
                    this.followTrajectory(
                            this.trajectoryBuilder(this.getPoseEstimate())
                                    .strafeLeft(sensorDistance - PlaceDistance,
                                            SampleMecanumDrive.getVelocityConstraint(ChassisSpeed.QuickPlaceSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(ChassisSpeed.QuickPlaceAccel)
                                    )
                                    .build()
                    );
                } else if (alignStrafe == Chassis.PoleAlign.Right) {
                    this.followTrajectory(
                            this.trajectoryBuilder(this.getPoseEstimate())
                                    .strafeRight(sensorDistance - PlaceDistance,
                                            SampleMecanumDrive.getVelocityConstraint(ChassisSpeed.QuickPlaceSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(ChassisSpeed.QuickPlaceAccel)
                                    )
                                    .build()
                    );
                }
            } else if (sensorDistance < PlaceDistance && PlaceDistance - sensorDistance > 0.01) {
                if (alignStrafe == Chassis.PoleAlign.Left) {
                    this.followTrajectory(
                            this.trajectoryBuilder(this.getPoseEstimate())
                                    .strafeRight(PlaceDistance - sensorDistance,
                                            SampleMecanumDrive.getVelocityConstraint(ChassisSpeed.QuickPlaceSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(ChassisSpeed.QuickPlaceAccel)
                                    )
                                    .build()
                    );
                } else if (alignStrafe == Chassis.PoleAlign.Right) {
                    this.followTrajectory(
                            this.trajectoryBuilder(this.getPoseEstimate())
                                    .strafeLeft(PlaceDistance - sensorDistance,
                                            SampleMecanumDrive.getVelocityConstraint(ChassisSpeed.QuickPlaceSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(ChassisSpeed.QuickPlaceAccel)
                                    )
                                    .build()
                    );
                }
            }

            return true;
        } else {
            this.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0,
                            0
                    )
            );
            return true;
        }
    } //end of autoplace

    public static class SmartAlignData {
        public ArrayList<AlignPos> distances = new ArrayList<>(); //list of distances and positions from the sensors

        public boolean sawPole = false; //if the robot saw the pole
        public boolean gotData = false; //if the robot finished getting data from the sensors

        public static int getBestIndex(ArrayList<AlignPos> sortedAlignData) {
            List<Double> sortedSensorDistances = sortedAlignData.stream() //streams the list of AlignPos
                    .map(alignPos -> alignPos.SensorDistance) //makes the list all the sensor distances
                    .collect(Collectors.toList()); //collects the list

            for (int i = 0; i < sortedSensorDistances.size() - 1; i++) { //iterates from min to max distances
                if (sortedSensorDistances.get(i + 1) - sortedSensorDistances.get(i) < SensorDistances.OutlierMargin) { //if the next distance is within the margin of the current distance
                    return i; //return the index of the current distance (the best distance)
                }
                //continue if found outlier
            }
            return 0; //if no outliers or best pos not found, return the first index
        }
    }

    public static class AlignPos {
        public Pose2d Position; //position of the robot when the sensor detected the pole
        public double SensorDistance; //distance the sensor detected the pole

        public AlignPos(Pose2d position, double sensorDistance) {
            this.Position = position;
            this.SensorDistance = sensorDistance;
        }
    }

    private SmartAlignData smartAlignData = new SmartAlignData();

    public final void smartAlignReset() {
        this.smartAlignData = new SmartAlignData();
    }

    public final SmartAlignData getSmartAlignData() {
        return this.smartAlignData;
    }

    /**
     * @param leftSensor  left distance sensor
     * @param rightSensor right distance sensor
     * @param alignDrive  direction to align with pole
     * @param alignStrafe side pole is on
     * @return true if aligned, false if not
     * <p>
     * Aligns with pole using distance sensors, does this in order:
     * 1. Drive forward/backward until distance sensor detects pole
     * 2. Keep driving forward/backward until distance sensors loses pole
     * 3. Keep track of each distance when detecting pole and the x value of the robot when it read that distance
     * 4. Use the x values and the distance to calculate the distance the robot needs to drive to align with the pole
     * 5. Drive to the calculated distance
     * 6. Return true because it is aligned
     */
    public final boolean smartAlign(DistanceSensor leftSensor, DistanceSensor rightSensor, Chassis.PoleAlign alignDrive, Chassis.PoleAlign alignStrafe) {
        this.update(); //updates the auto position

        double sensorDistance = alignStrafe == Chassis.PoleAlign.Left ? leftSensor.getDistance(DistanceUnit.INCH) : rightSensor.getDistance(DistanceUnit.INCH); //gets the distance from the sensor

        if (smartAlignData.sawPole && smartAlignData.gotData) { // if we have seen the pole and have the data
            // Create a new list of sensor distances
            ArrayList<AlignPos> sortedAlignData = smartAlignData.distances.stream() //streams the list of AlignPos
                    .sorted(Comparator.comparingDouble(alignPos -> alignPos.SensorDistance)) //compares the sensor distances to sort list
                    .collect(Collectors.toCollection(ArrayList::new)); //back to list

            AlignPos bestAlignPos = sortedAlignData.get(
                    SmartAlignData.getBestIndex(sortedAlignData) //gets the best align position
            ); //gets the best align position

            double PlaceDistance = alignStrafe == Chassis.PoleAlign.Left ? SensorDistances.LeftPlaceDistance : SensorDistances.RightPlaceDistance; //gets the place distance depending on aligning direction

            //calculates the distance to align to the pole, positive means driving to left, negative means driving to right
            double dist = alignStrafe == Chassis.PoleAlign.Left ? (bestAlignPos.SensorDistance - PlaceDistance) : -(bestAlignPos.SensorDistance - PlaceDistance);

            double rotationalOffset = alignStrafe == Chassis.PoleAlign.Left ? 90 : -90; //gets the rotational offset depending on aligning direction
            double headingSin = Math.sin(bestAlignPos.Position.getHeading() + Math.toRadians(rotationalOffset)); //gets the sin of the heading
            double headingCos = Math.cos(bestAlignPos.Position.getHeading() + Math.toRadians(rotationalOffset)); //gets the cos of the heading
            Pose2d calculatedAlignPos = new Pose2d( //calculates the position to align to
                    bestAlignPos.Position.getX() + dist * headingCos, //calculates the x value
                    bestAlignPos.Position.getY() + dist * headingSin, //calculates the y value
                    bestAlignPos.Position.getHeading() //keeps the same heading
            );

            this.followTrajectory( // drive to calculated position
                    this.trajectoryBuilder(this.getPoseEstimate())
                            .lineToLinearHeading(calculatedAlignPos)
                            .build()
            );

//            this.smartAlignReset(); //reset smart align data for next time
            return true; //finished aligning
        } else if (smartAlignData.sawPole) {
            if (
                    (smartAlignData.distances.size() > 2 && //if there are at least 2 distances (also prevents index out of bounds)
                            sensorDistance < smartAlignData.distances.get(smartAlignData.distances.size() - 1).SensorDistance + SensorDistances.LosingPoleMargin //if sensor distance is decreasing
                            && smartAlignData.distances.get(smartAlignData.distances.size() - 1).SensorDistance < smartAlignData.distances.get(smartAlignData.distances.size() - 2).SensorDistance + SensorDistances.StillLosingPoleMargin //if sensor distance was previously decreasing
                    )
                            || sensorDistance < SensorDistances.DetectAmount) { //if the sensor completely loses the pole
                this.setWeightedDrivePower(
                        new Pose2d(
                                alignDrive == Chassis.PoleAlign.Forward ? ChassisSpeed.FineAlignSpeed : -ChassisSpeed.FineAlignSpeed,
                                0,
                                0
                        )
                );
                smartAlignData.distances.add(new AlignPos(this.getPoseEstimate(), sensorDistance));
            } else { //stop moving if all data collected
                this.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );
                smartAlignData.gotData = true; //got data
            }
        } else if (sensorDistance < SensorDistances.DetectAmount) { //looking for pole
            this.setWeightedDrivePower(
                    new Pose2d(
                            alignDrive == Chassis.PoleAlign.Forward ? ChassisSpeed.FineAlignSpeed : -ChassisSpeed.FineAlignSpeed,
                            0,
                            0
                    )
            );
            smartAlignData.sawPole = true; //found pole
            smartAlignData.distances.add(new AlignPos(this.getPoseEstimate(), sensorDistance)); //log first pole position
        } else {
            this.setWeightedDrivePower(
                    new Pose2d(
                            alignDrive == Chassis.PoleAlign.Forward ? ChassisSpeed.AlignSpeed : -ChassisSpeed.AlignSpeed,
                            0,
                            0
                    )
            );
        }

        return false; //didn't finish aligning yet
    }
}

