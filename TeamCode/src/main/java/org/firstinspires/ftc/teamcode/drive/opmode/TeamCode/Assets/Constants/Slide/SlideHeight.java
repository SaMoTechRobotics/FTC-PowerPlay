package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideHeight {

  /**
   * Heights of slide to line up with heights of different poles or ground as inches
   */
  public static double HighPole = 45; //4000
  public static double MidPole = 32; //2800
  public static double LowPole = 22; //2000
  public static double Ground = 0;

  /**
   * Max and min heights of slide as percentages
   */
  public static double MaxHeight = 46;
  public static double MinHeight = 0;

  /**
   * The base offset height, where slide is mounted relative to ground
   */
  public static double BaseHeight = 4;

  /**
   * The minimum height for the arm to be allowed to rotate
   */
  public static double SafetyHeight = 20;

  /**
   * The safety margin to move arm to center preventing failure in robot
   */
  public static double SafetyMargin = 18;

  /**
   * The ticks per revolution of the slide motor
   */
  public static final double TicksPerRev = 537.7;

  /**
   * The diameter of the pulley on the slide in inches
   */
  public static final double RevDiameter = 1.673228;

  /**
   * The ticks per inch of the slide moving up
   */
  public static final double TicksPerInch = TicksPerRev / (RevDiameter * Math.PI);
}
