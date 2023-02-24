package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(36, -12, Math.toRadians(45));

        double dist = 20;

        double forwardOffset = 10;

        // Calculate the end pose by transforming to the left by dist relative to heading, make it work with any heading
        double perpendicularHeadingSin = Math.sin(startPose.getHeading() + Math.toRadians(90));
        double perpendicularHeadingCos = Math.cos(startPose.getHeading() + Math.toRadians(90));

        double headingSin = Math.sin(startPose.getHeading());
        double headingCos = Math.cos(startPose.getHeading());

        Pose2d endPose = new Pose2d(
                startPose.getX() + (dist * perpendicularHeadingCos) + (forwardOffset * headingCos),
                startPose.getY() + (dist * perpendicularHeadingSin) + (forwardOffset * headingSin),
                startPose.getHeading()
        );

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(500), Math.toRadians(180), 13.63)
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(endPose)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}