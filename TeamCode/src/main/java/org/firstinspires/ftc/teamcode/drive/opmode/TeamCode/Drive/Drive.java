package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {


    private Slide Slide;
    private Chassis Chassis;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        FtcDashboard dashboard = FtcDashboard.getInstance();
        
        Slide = new Slide(
            hardwareMap.get(DcMotor.class, "slide")
        );

        Chassis = new Chassis(
            hardwareMap.get(DcMotor.class, "frontLeft"),
            hardwareMap.get(DcMotor.class, "frontRight"),
            hardwareMap.get(DcMotor.class, "backLeft"),
            hardwareMap.get(DcMotor.class, "backRight")
        );
       
        waitForStart();
        
        while (opModeIsActive()) {
           
        }
    }
}