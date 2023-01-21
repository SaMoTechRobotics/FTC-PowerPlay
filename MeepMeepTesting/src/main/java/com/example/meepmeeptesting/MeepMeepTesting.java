package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(55.5, 30, Math.toRadians(50), Math.toRadians(180), 13.63)
//                .setStartPose(new Pose2d(36, -63, Math.toRadians(90)))
//                .setDimensions(13, 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(36, -64, Math.toRadians(90)))
//                                .forward(40)
//                                .addDisplacementMarker(22, () -> {
//                                    //Reading signal sleeve
//                                })
//                                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(0)))
//                                .build()
//                );

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55.5, 30, Math.toRadians(90), Math.toRadians(180), 13.63)
//                .setStartPose(new Pose2d(64, -63, Math.toRadians(90)))
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(64, -63, Math.toRadians(90)))
                                .strafeTo(new Vector2d(60, -60))
                                .forward(35)
                                .splineToLinearHeading(new Pose2d(48, -12, Math.toRadians(180)), Math.toRadians(180))
                                .forward(24)
                                .splineToLinearHeading(new Pose2d(10, -25, Math.toRadians(270)), Math.toRadians(270))
                                .forward(25)
                                .splineToLinearHeading(new Pose2d(24, -60, Math.toRadians(0)), Math.toRadians(0))
                                .forward(10)
                                .lineToLinearHeading(new Pose2d(50, -60, Math.toRadians(90)))
                                .strafeTo(new Vector2d(64, -63))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}