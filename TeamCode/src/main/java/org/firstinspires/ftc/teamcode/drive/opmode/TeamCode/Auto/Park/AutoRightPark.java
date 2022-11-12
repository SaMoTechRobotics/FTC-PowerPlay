package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "AutoRightPark", group = "AutoRight")
@Disabled
public class AutoRightPark extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    waitForStart();

    if (isStopRequested()) return;


    /*
    private val startPose = Pose2d(-65.0, -35.0, 0.0.toRadians)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)

        val pos = 3;

        builder1
            .forward(30.0)

        val builder2 = TrajectoryBuilder(Pose2d(-15.0, -35.0), startPose.heading, combinedConstraints)
        builder2.strafeLeft(10.0)

        val builder3 = TrajectoryBuilder(Pose2d(-15.0, -25.0), startPose.heading, combinedConstraints)
        builder3.strafeRight(10.0)

        val builder4 = TrajectoryBuilder(Pose2d(-15.0, -35.0), startPose.heading, combinedConstraints)
        builder4.back(20.0);


        val lastPosBuilder = TrajectoryBuilder(Pose2d(-35.0, -35.0), startPose.heading, combinedConstraints)

        if(pos == 1) {
            lastPosBuilder.strafeLeft(24.0);
        } else if (pos == 3) {
            lastPosBuilder.strafeRight(24.0);
        }

        // Small Example Routine
        // builder1
        //    .splineTo(Vector2d(10.0, 10.0), 0.0)
        //    .splineTo(Vector2d(35.0, 35.0), 90.0)
        //    .splineTo(Vector2d(0.0, -50.0), 90.0)
        //    .splineTo(Vector2d(50.0, -50.0), 90.0)
        //    .splineTo(Vector2d(-50.0,0.0),180.0);

        list.add(builder1.build())
        //list.add(builder2.build())
        //list.add(builder3.build())
        //list.add(builder4.build())
        if(pos == 1 || pos == 3 )list.add(lastPosBuilder.build())
     
     */
  }
}
