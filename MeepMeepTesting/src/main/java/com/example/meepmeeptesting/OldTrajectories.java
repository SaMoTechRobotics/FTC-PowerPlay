/**
 * Circles around the field
 * <p>
 * Trajectories
 * <p>
 * Trajectories
 * <p>
 * Trajectories
 */
//RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
//        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//        .setConstraints(55.5, 30, Math.toRadians(90), Math.toRadians(180), 13.63)
////                .setStartPose(new Pose2d(64, -63, Math.toRadians(90)))
//        .setDimensions(13, 15)
//        .followTrajectorySequence(drive ->
//        drive.trajectorySequenceBuilder(new Pose2d(64, -63, Math.toRadians(90)))
//        .strafeTo(new Vector2d(60, -60))
//        .forward(35)
//        .splineToLinearHeading(new Pose2d(48, -12, Math.toRadians(180)), Math.toRadians(180))
//        .forward(96)
//        .splineToLinearHeading(new Pose2d(-60, -25, Math.toRadians(270)), Math.toRadians(270))
//        .forward(25)
//        .splineToLinearHeading(new Pose2d(-48, -60, Math.toRadians(0)), Math.toRadians(0))
//        .forward(86)
//        .lineToLinearHeading(new Pose2d(60, -60, Math.toRadians(90)))
//        .strafeTo(new Vector2d(64, -63))
//        .build()
//        );

/*
 * AutoRightPro
 */
/*

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

*/

/**
 * Trajectories
 */