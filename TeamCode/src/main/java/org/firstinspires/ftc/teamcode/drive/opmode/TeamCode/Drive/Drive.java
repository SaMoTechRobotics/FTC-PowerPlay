package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends LinearOpMode {


    private Slide Slide;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        FtcDashboard dashboard = FtcDashboard.getInstance();
        
        Slide = new Slide(
            hardwareMap.get(DcMotor.class, "slideLeft"),
            hardwareMap.get(DcMotor.class, "slideRight")
        );
        
        waitForStart();
        
        while (opModeIsActive()) {
           
        }
    }
}