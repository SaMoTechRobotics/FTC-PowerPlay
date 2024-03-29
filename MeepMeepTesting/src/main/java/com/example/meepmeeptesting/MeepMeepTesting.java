package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(40.5, -63, Math.toRadians(270));

        double deliverConeTime = 1;

        double pickupConeTime = 0.3;

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(500), Math.toRadians(180), 13.63)
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .setReversed(true) //reverse splines
                                        .splineToLinearHeading(new Pose2d(35, -60, Math.toRadians(270)), Math.toRadians(90)) //clear the wall
                                        .splineTo(new Vector2d(35, -46), Math.toRadians(90)) //drive around ground junction
                                        .splineTo(new Vector2d(36, -10), Math.toRadians(90)) //drive to first high pole
//                                        .splineTo(new Vector2d(28, -10), Math.toRadians(180)) //drive to first high pole and turn
                                        .setReversed(false) //undo reverse
                                        .lineToLinearHeading(new Pose2d(31, -13, Math.toRadians(0)))
                                        //drive to first high pol
//                                        .turn(Math.toRadians(90)) //turn to face stack

//                                        .lineToLinearHeading(new Pose2d(28, -10, Math.toRadians(0))) //back up to high pole
                                        .waitSeconds(deliverConeTime) //deliver cone
//                                        .splineTo(new Vector2d(44, -12.7), Math.toRadians(-11)) //drive away from high pole
//                                        .splineTo(new Vector2d(59, -12), Math.toRadians(2)) //drive around ground junction
//                                        .splineTo(new Vector2d(64, -12), Math.toRadians(0)) //drive to stack
                                        .splineToLinearHeading(new Pose2d(30, -13, Math.toRadians(0)), Math.toRadians(0))
                                        .splineTo(new Vector2d(48, -12), Math.toRadians(0))
                                        .splineTo(new Vector2d(64, -12), Math.toRadians(0))

                                        .waitSeconds(pickupConeTime) //pickup cone
                                        .setReversed(true)
                                        .splineTo(new Vector2d(48, -12), Math.toRadians(180))
                                        .splineTo(new Vector2d(28, -10), Math.toRadians(180)) //drive to stack
                                        .setReversed(false)
                                        .waitSeconds(deliverConeTime) //deliver cone
                                        .splineTo(new Vector2d(48, -12), Math.toRadians(0))
                                        .splineTo(new Vector2d(64, -12), Math.toRadians(0))

                                        .waitSeconds(pickupConeTime) //pickup cone
                                        .setReversed(true)
                                        .splineTo(new Vector2d(48, -12), Math.toRadians(180))
                                        .splineTo(new Vector2d(28, -10), Math.toRadians(180)) //drive to stack
                                        .setReversed(false)
                                        .waitSeconds(deliverConeTime) //deliver cone

                                        .splineTo(new Vector2d(48, -12), Math.toRadians(0))
                                        .splineTo(new Vector2d(64, -12), Math.toRadians(0))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}