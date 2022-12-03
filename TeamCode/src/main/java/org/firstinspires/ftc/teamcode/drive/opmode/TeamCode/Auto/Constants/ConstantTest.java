package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class ConstantTest {
    public static String TestString = "Hello World";
    public static int TestInt = 5;
    public static double TestDouble = 5.5;
    public static boolean TestBoolean = true;
    public static Pose2d TestPose2d = new Pose2d(10, 20, Math.toRadians(90));
    public static int[] TestIntArray = {1, 2, 3, 4, 5};
}
