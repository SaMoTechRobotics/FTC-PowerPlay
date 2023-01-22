package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(36, -63, Math.toRadians(270));

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 75, Math.toRadians(90), Math.toRadians(180), 13.63)
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(35)
                                .addDisplacementMarker(22, () -> {
                                    //Reading signal sleeve
                                })
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(34, -25, Math.toRadians(270)))
                                .waitSeconds(0.8)
                                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(56, -12, Math.toRadians(0)))
                                .waitSeconds(0.5) //start of 2 cone
                                .forward(7.5)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(5, -14, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .back(5) //auto align
                                .waitSeconds(0.8)
                                .strafeLeft(3)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(56, -12, Math.toRadians(0))) //end of 2 cone
                                .waitSeconds(0.5) //start of 3 cone
                                .forward(7.5)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(5, -14, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .back(5) //auto align
                                .waitSeconds(0.8)
                                .strafeLeft(3)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(56, -12, Math.toRadians(0))) //end of 3 cone
                                .waitSeconds(0.5) //start of 4 cone
                                .forward(7.5)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(30, -10, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .back(5) //auto align
                                .waitSeconds(0.8)
                                .strafeRight(3)
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(56, -12, Math.toRadians(0))) //end of 4 cone
                                .build()
                );

        /**
         * Parking - 20
         * 1 Mid Cone - 8
         * 2 Far High Cone = 20
         * 1 High Cone = 10
         * Total: 58
         */


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}