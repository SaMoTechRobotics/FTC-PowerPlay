package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Field {

  public static final double TILE_SIZE = 24.0;

  public static String ChassisColor = "#FF0000";
  public static String ArmColor = "#00FF00";
  public static String DistanceSensorColor = "#0000FF";

  private static final Canvas _getCanvas() {
    return new TelemetryPacket().fieldOverlay();
  }

  public final void drawChassis(ChassisPosition position) {
    Canvas canvas = _getCanvas();
    canvas.setFill(ChassisColor);
    double x = position.getX();
    double y = position.getY();
    double angle = position.getRotation();
    double width = ChassisConstants.FullWidth;
    double length = ChassisConstants.FullLength;
    double x1 =
      x + (width / 2) * Math.cos(angle) - (length / 2) * Math.sin(angle);
    double y1 =
      y + (width / 2) * Math.sin(angle) + (length / 2) * Math.cos(angle);
    double x2 =
      x + (width / 2) * Math.cos(angle) + (length / 2) * Math.sin(angle);
    double y2 =
      y + (width / 2) * Math.sin(angle) - (length / 2) * Math.cos(angle);
    double x3 =
      x - (width / 2) * Math.cos(angle) + (length / 2) * Math.sin(angle);
    double y3 =
      y - (width / 2) * Math.sin(angle) - (length / 2) * Math.cos(angle);
    double x4 =
      x - (width / 2) * Math.cos(angle) - (length / 2) * Math.sin(angle);
    double y4 =
      y - (width / 2) * Math.sin(angle) + (length / 2) * Math.cos(angle);
    canvas.fillPolygon(
      new double[] { x1, x2, x3, x4 },
      new double[] { y1, y2, y3, y4 }
    );
  }

  public final void drawDistanceSensor(
    double relative_x,
    double relative_y,
    double angle,
    double distance
  ) {
    Canvas canvas = _getCanvas();
    canvas.setStroke(DistanceSensorColor);
    double x = ChassisConstants.FullWidth / 2 + relative_x;
    double y = ChassisConstants.FullLength / 2 + relative_y;
    double x1 = x + distance * Math.cos(angle);
    double y1 = y + distance * Math.sin(angle);
    canvas.strokeLine(x, y, x1, y1);
  }
}
