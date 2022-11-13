package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
/**
 * This class contains all the constants for the chassis
 * All measurements are in inches
 */
public class ChassisConstants {

  /**
   * The full dimensions of the robot measured from outside to outside
   */
  public static double FullWidth = 13.0;
  public static double FullLength = 15.0;

  /**
   * Wheel diameter in inches
   */
  public static double WheelDiameter = 4.0;

  /**
   * The width of the wheel in inches
   */
  public static double WheelWidth = 1.0;

  /**
   * The gear ratio of the motor
   */
  public static double GearRatio = 1;

  /**
   * The ticks per revolution of the motor
   */
  public static final double TicksPerRev = 537.6;

  /**
   * The circumference of the wheel in inches
   */
  public static final double WheelCircumference = WheelDiameter * Math.PI;

  /**
   * The ticks per inch of the motor
   */
  public static final double TicksPerInch =
    TicksPerRev * GearRatio / WheelCircumference;
}
