package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class DashboardPreview {
    public static void updateField() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay()
            .setFill("blue")
            .fillRect(0, 0, RobotDimensions.Base.Width, RobotDimensions.Base.Length);

        dashboard.sendTelemetryPacket(packet);
    }
}
