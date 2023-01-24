package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(36, -27, Math.toRadians(270));


        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 75, Math.toRadians(90), Math.toRadians(180), 13.63)
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
//                                .splineTo(new Vector2d(36, 0), Math.toRadians(270))
//                                .splineTo(new Vector2d(40, -11), Math.toRadians(315))
//                                .splineTo(new Vector2d(50, -12), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(36, -5, Math.toRadians(270)), Math.toRadians(270))
//                                        .splineToLinearHeading(new Pose2d(40, -11, Math.toRadians(315)), Math.toRadians(315))
                                        .splineToLinearHeading(new Pose2d(50, -12, Math.toRadians(0)), Math.toRadians(0))
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