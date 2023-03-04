package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(36, -24, Math.toRadians(90));
//
//        double dist = 4;
////
//        double forwardOffset = 0;
////
//        // Calculate the end pose by transforming to the left by dist relative to heading, make it work with any heading
//        double perpendicularHeadingSin = Math.sin(startPose.getHeading() + Math.toRadians(90));
//        double perpendicularHeadingCos = Math.cos(startPose.getHeading() + Math.toRadians(90));
//
//        double headingSin = Math.sin(startPose.getHeading());
//        double headingCos = Math.cos(startPose.getHeading());
//
//        Pose2d endPose = new Pose2d(
//                startPose.getX() + (dist * perpendicularHeadingCos) + (forwardOffset * headingCos),
//                startPose.getY() + (dist * perpendicularHeadingSin) + (forwardOffset * headingSin),
//                startPose.getHeading()
//        );

// Define the angle of rotation in radians
        double rotationAngle = Math.toRadians(-90); // 45 degrees clockwise

// Define the current point
        Vector2d currentPoint = new Vector2d(startPose.getX(), startPose.getY());

// Define the pivot point as a Pose2d object
        Pose2d centerPoint = new Pose2d(52, -16, 0);

// Calculate the distance between the current point and the pivot point
        double dist = currentPoint.minus(centerPoint.vec()).norm();

// Translate the current point relative to the pivot point
        Vector2d translatedPoint = currentPoint.minus(centerPoint.vec());

// Calculate the normalized vector
        double mag = translatedPoint.norm();
        Vector2d normalizedPoint = new Vector2d(translatedPoint.getX() / mag, translatedPoint.getY() / mag);

// Multiply the normalized vector by the desired distance
        Vector2d scaledPoint = normalizedPoint.times(dist);

// Apply the rotation transformation
        Vector2d rotatedPoint = scaledPoint.rotated(rotationAngle);

// Translate the rotated point back to the original coordinate system
        Pose2d finalPoint = new Pose2d(rotatedPoint.plus(centerPoint.vec()), startPose.getHeading() + rotationAngle);

        int SIDE = -1; //-1 for left, 1 for right

        double FINAL_ROT = 180; //180 for left, 0 for right

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(500), Math.toRadians(180), 13.63)
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(finalPoint)
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}