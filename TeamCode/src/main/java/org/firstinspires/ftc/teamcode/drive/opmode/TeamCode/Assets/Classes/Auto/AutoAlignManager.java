package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.ChassisSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.AlignDataParams;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorDistances;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class AutoAlignManager {

    public static class SmartAlignData {
        public ArrayList<AlignPos> distances = new ArrayList<>(); //list of distances and positions from the sensors

        public boolean sawPole = false; //if the robot saw the pole
        public boolean gotData = false; //if the robot finished getting data from the sensors

        public Pose2d calculatedPosition = new Pose2d(); //the calculated position of the robot
        public Pose2d calculatedCenterPosition = new Pose2d(); //the calculated position of the robot

        public static int getBestIndex(ArrayList<AlignPos> sortedAlignData) {
            List<Double> sortedSensorDistances = sortedAlignData.stream() //streams the list of AlignPos
                    .map(alignPos -> alignPos.SensorDistance) //makes the list all the sensor distances
                    .collect(Collectors.toList()); //collects the list

            for (int i = 0; i < sortedSensorDistances.size() - 1; i++) { //iterates from min to max distances
                if (sortedSensorDistances.get(i + 1) - sortedSensorDistances.get(i) < AlignDataParams.OutlierMargin) { //if the next distance is within the margin of the current distance
                    return i; //return the index of the current distance (the best distance)
                }
                //continue if found outlier
            }
            return 0; //if no outliers or best pos not found, return the first index
        }

        public static Pose2d getCalculatedPosition(AlignPos bestAlignPos, Chassis.PoleAlign alignStrafe) {
            double PlaceDistance = alignStrafe == Chassis.PoleAlign.Left ? SensorDistances.LeftPlaceDistance : SensorDistances.RightPlaceDistance; //gets the place distance depending on aligning direction

            //calculates the distance to align to the pole, positive means driving to left, negative means driving to right
            double dist = (bestAlignPos.SensorDistance - PlaceDistance);

            double rotationalOffset = alignStrafe == Chassis.PoleAlign.Left ? 90 : -90; //gets the rotational offset depending on aligning direction
            double perpendicularHeadingSin = Math.sin(bestAlignPos.Position.getHeading() + Math.toRadians(rotationalOffset)); //gets the sin of the heading with the offset
            double perpendicularHeadingCos = Math.cos(bestAlignPos.Position.getHeading() + Math.toRadians(rotationalOffset)); //gets the cos of the heading with the offset

            double offset = alignStrafe == Chassis.PoleAlign.Left ? AlignDataParams.LeftForwardOffset : AlignDataParams.RightForwardOffset; //gets the offset depending on aligning direction

            double headingSin = Math.sin(bestAlignPos.Position.getHeading()); //gets the sin of the heading
            double headingCos = Math.cos(bestAlignPos.Position.getHeading()); //gets the cos of the heading

            return new Pose2d( //calculates the position to align to
                    bestAlignPos.Position.getX() + (dist * perpendicularHeadingCos) + (offset * headingCos), //calculates the x value
                    bestAlignPos.Position.getY() + (dist * perpendicularHeadingSin) + (offset * headingSin), //calculates the y value
                    bestAlignPos.Position.getHeading() //keeps the same heading
            );
        }

        public static Pose2d getCalculatedCenterPosition(AlignPos bestAlignPos, Chassis.PoleAlign alignStrafe, Pose2d currentPose) {
            double PlaceDistance = alignStrafe == Chassis.PoleAlign.Left ? SensorDistances.LeftCenterDistance : SensorDistances.RightCenterDistance; //gets the place distance depending on aligning direction

            //calculates the distance to align to the pole, positive means driving to left, negative means driving to right
            double dist = (bestAlignPos.SensorDistance - PlaceDistance);

            double rotationalOffset = alignStrafe == Chassis.PoleAlign.Left ? 90 : -90; //gets the rotational offset depending on aligning direction
            double perpendicularHeadingSin = Math.sin(bestAlignPos.Position.getHeading() + Math.toRadians(rotationalOffset)); //gets the sin of the heading with the offset
            double perpendicularHeadingCos = Math.cos(bestAlignPos.Position.getHeading() + Math.toRadians(rotationalOffset)); //gets the cos of the heading with the offset

            double offset = Math.abs(bestAlignPos.Position.getX() - currentPose.getX()); //gets the offset depending on aligning direction

            double headingSin = Math.sin(bestAlignPos.Position.getHeading()); //gets the sin of the heading
            double headingCos = Math.cos(bestAlignPos.Position.getHeading()); //gets the cos of the heading

            return new Pose2d( //calculates the position to align to
                    bestAlignPos.Position.getX() + (dist * perpendicularHeadingCos) + (offset * headingCos), //calculates the x value
                    bestAlignPos.Position.getY() + (dist * perpendicularHeadingSin) + (offset * headingSin), //calculates the y value
                    bestAlignPos.Position.getHeading() //keeps the same heading
            );
        }
//
//        public static Pose2d getCalculatedCenterPosition(AlignPos bestAlignPos, Chassis.PoleAlign alignStrafe) {
//            double PlaceDistance = alignStrafe == Chassis.PoleAlign.Left ? SensorDistances.LeftCenterDistance : SensorDistances.RightCenterDistance;
//
//            double dist = alignStrafe == Chassis.PoleAlign.Left ? (bestAlignPos.SensorDistance - PlaceDistance) : -(bestAlignPos.SensorDistance - PlaceDistance);
//
//            double offset = alignStrafe == Chassis.PoleAlign.Left ? AlignDataParams.LeftForwardOffset : AlignDataParams.RightForwardOffset;
//
//            // Define the current point
//            Vector2d currentPoint = new Vector2d(bestAlignPos.Position.getX(), bestAlignPos.Position.getY());
//
//            // Define the pivot point as a Pose2d object
//            Pose2d centerPoint = new Pose2d(bestAlignPos.SensorDistance, 0, 0);
//
//            // Calculate the distance between the current point and the pivot point
//            double distance = currentPoint.minus(centerPoint.vec()).norm();
//
//            // Translate the current point relative to the pivot point
//            Vector2d translatedPoint = currentPoint.minus(centerPoint.vec());
//
//            // Calculate the normalized vector
//            double mag = translatedPoint.norm();
//            Vector2d normalizedPoint = new Vector2d(translatedPoint.getX() / mag, translatedPoint.getY() / mag);
//
//            // Multiply the normalized vector by the desired distance
//            Vector2d scaledPoint = normalizedPoint.times(distance);
//
//            // Define the rotation angle in radians
//            double rotationAngle = Math.toRadians(180); // 180 degrees clockwise
//
//            // Apply the rotation transformation
//            Vector2d rotatedPoint = scaledPoint.rotated(rotationAngle);
//
//            // Translate the rotated point back to the original coordinate system
//            Vector2d finalPoint = rotatedPoint.plus(centerPoint.vec());
//
//            return new Pose2d(
//                    finalPoint.getX() + (dist * Math.cos(rotationAngle)) + (offset * Math.cos(0)),
//                    finalPoint.getY() + (dist * Math.sin(rotationAngle)) + (offset * Math.sin(0)),
//                    0
//            );
//        }
    }

    public static class AlignPos {
        public Pose2d Position; //position of the robot when the sensor detected the pole
        public double SensorDistance; //distance the sensor detected the pole

        public AlignPos(Pose2d position, double sensorDistance) {
            this.Position = position;
            this.SensorDistance = sensorDistance;
        }
    }

    private static SmartAlignData smartAlignData = new SmartAlignData();

    public static void smartAlignReset() {
        smartAlignData = new SmartAlignData();
    }

    public static SmartAlignData getSmartAlignData() {
        return smartAlignData;
    }

    public static Pose2d getCalculatedPosition() {
        return smartAlignData.calculatedPosition;
    }

    public static Pose2d getCalculatedCenterPosition() {
        return smartAlignData.calculatedCenterPosition;
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
    public static boolean smartAlign(SampleMecanumDrive drive, DistanceSensor leftSensor, DistanceSensor rightSensor, Chassis.PoleAlign alignDrive, Chassis.PoleAlign alignStrafe, boolean... isCenter) {
        drive.update(); //updates the auto position

        double sensorDistance = alignStrafe == Chassis.PoleAlign.Left ? leftSensor.getDistance(DistanceUnit.INCH) : rightSensor.getDistance(DistanceUnit.INCH); //gets the distance from the sensor

        if (smartAlignData.sawPole && smartAlignData.gotData) { // if we have seen the pole and have the data
            // Create a new list of sensor distances
            ArrayList<AlignPos> sortedAlignData = smartAlignData.distances.stream() //streams the list of AlignPos
                    .sorted(Comparator.comparingDouble(alignPos -> alignPos.SensorDistance)) //compares the sensor distances to sort list
                    .collect(Collectors.toCollection(ArrayList::new)); //back to list

            AlignPos bestAlignPos = sortedAlignData.get(
                    SmartAlignData.getBestIndex(sortedAlignData) //gets the best align position
            ); //gets the best align position

            smartAlignData.calculatedPosition = SmartAlignData.getCalculatedPosition(bestAlignPos, alignStrafe); //gets the calculated position to align with the pole
            smartAlignData.calculatedCenterPosition = SmartAlignData.getCalculatedCenterPosition(bestAlignPos, alignStrafe, drive.getPoseEstimate()); //gets the calculated position to align with center of tile

            drive.followTrajectory( // drive to calculated position
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(
                                    (isCenter.length > 0 && isCenter[0]) ? //if isCenter is true
                                            smartAlignData.calculatedCenterPosition :
                                            smartAlignData.calculatedPosition
                            )
                            .build()
            );

//            this.smartAlignReset(); //reset smart align data for next time
            return true; //finished aligning
        } else if (smartAlignData.sawPole) {
//            boolean finishedDataCollection = smartAlignData.distances.get(smartAlignData.distances.size() - 1).SensorDistance > SensorDistances.DetectAmount; //temp var to check if data collection is finished, starts as false
//            if (!finishedDataCollection) {
//                for (int i = 0; i < AlignDataParams.LosingPoleDataAmount; i++) { //checks if the sensor is losing the pole
//                    if (smartAlignData.distances.size() > i + 1) { //if the distance list is long enough
//                        if (smartAlignData.distances.get(smartAlignData.distances.size() - 1 - i).SensorDistance < //if the current distance is less than the previous distance
//                                smartAlignData.distances.get(smartAlignData.distances.size() - 2 - i).SensorDistance + AlignDataParams.LosingPoleMargin) { //if the current distance is less than the previous distance plus the margin
//                            finishedDataCollection = true; //data collection is finished because the sensor is losing the pole
//                            break;
//                        }
//                    }
//                }
//            }
//            if ((!finishedDataCollection && smartAlignData.distances.size() > AlignDataParams.MinimumDataAmount)
//                    || sensorDistance < SensorDistances.DetectAmount) { //if the sensor completely loses the pole

            if (smartAlignData.distances.size() > AlignDataParams.MinimumDataAmount && //collected enough data
                    (sensorDistance < SensorDistances.DetectAmount || //sensor lost pole completely
                            sensorDistance - AlignDataParams.LosingPoleMargin > smartAlignData.distances.get(smartAlignData.distances.size() - 1).SensorDistance)) { //if the sensor is losing the pole
                drive.setWeightedDrivePower( //stop moving
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );
                smartAlignData.gotData = true; //got data
            } else { // continue driving
                drive.setWeightedDrivePower(
                        new Pose2d(
                                alignDrive == Chassis.PoleAlign.Forward ? ChassisSpeed.FineAlignSpeed : -ChassisSpeed.FineAlignSpeed,
                                0,
                                0
                        )
                );
                smartAlignData.distances.add(new AlignPos(drive.getPoseEstimate(), sensorDistance));
                RobotLog.d("SensorDistance: " + sensorDistance);
            }
        } else if (sensorDistance < SensorDistances.DetectAmount) { //looking for pole
            drive.setWeightedDrivePower(
                    new Pose2d(
                            alignDrive == Chassis.PoleAlign.Forward ? ChassisSpeed.FineAlignSpeed : -ChassisSpeed.FineAlignSpeed,
                            0,
                            0
                    )
            );
            smartAlignData.sawPole = true; //found pole
            smartAlignData.distances.add(new AlignPos(drive.getPoseEstimate(), sensorDistance)); //log first pole position
        } else {
            drive.setWeightedDrivePower(
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
