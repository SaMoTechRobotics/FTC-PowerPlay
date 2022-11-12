package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class StartPose {

  public static Pose2d LeftPose = new Pose2d(-65, 35, Math.toRadians(0));

  public static Pose2d RightPose = new Pose2d(-65, -35, Math.toRadians(0));
}
