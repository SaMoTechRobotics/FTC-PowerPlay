package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawPosition {

  //open: 40, close: 90
  /**
   * Max and min positions of the claw, ex: 0.0 to 1.0 resulting in 1.0 range
   */
  public static double Max = 1.0;
  public static double Min = 0.0;

  /**
   * The max open and close positions of the claw
   */
  public static double MaxOpen = 100;
  public static double MaxClosed = 0;

  /**
   * The open and close positions of the claw
   */
  public static double Open = 35;
  public static double Close = 60;

  public static double ConeDistance = 3.0;
}
