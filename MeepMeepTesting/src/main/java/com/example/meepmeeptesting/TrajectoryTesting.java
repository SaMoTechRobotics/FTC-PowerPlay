package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-40.5, -63, Math.toRadians(270));

//        double dist = 20;
//
//        double forwardOffset = 10;
//
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

        int SIDE = -1; //-1 for left, 1 for right

        double FINAL_ROT = 180; //180 for left, 0 for right

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(500), Math.toRadians(180), 13.63)
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .setReversed(true) //reverse splines
                                .splineToLinearHeading(new Pose2d(SIDE * 35, -60, Math.toRadians(270)), Math.toRadians(90)
                                ) //clear the wall
                                .splineTo(new Vector2d(SIDE * 35, -46), Math.toRadians(90)
                                ) //drive around ground junction
                                .splineTo(new Vector2d(SIDE * 35, -10), Math.toRadians(90)) //drive to first high pole and turn
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(SIDE * 28, -13, Math.toRadians(FINAL_ROT)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}