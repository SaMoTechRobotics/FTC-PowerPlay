package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro.Variants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.TargetPole;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro.AutoRightPro;

@Autonomous(name = "AutoRightProMid", group = "B")
public class AutoRightProMid extends AutoRightPro {
    private static class ConesToScore {
        public static int Count = 3;

        public static TargetPole Pole0 = TargetPole.CloseMid;
        public static TargetPole Pole1 = TargetPole.CloseMid;
        public static TargetPole Pole2 = TargetPole.CloseMid;
        public static TargetPole Pole3 = TargetPole.CloseMid;

        //Extra cones: typically don't count unless cones to score is > 3
        public static TargetPole Pole4 = TargetPole.CloseHigh;
        public static TargetPole Pole5 = TargetPole.CloseHigh;

        public static TargetPole[] getPoles() { //Returns an array of the poles to score on
            return new TargetPole[]{
                    Pole0,
                    Pole1,
                    Pole2,
                    Pole3,
                    Pole4,
                    Pole5
            };
        }
    }
}
