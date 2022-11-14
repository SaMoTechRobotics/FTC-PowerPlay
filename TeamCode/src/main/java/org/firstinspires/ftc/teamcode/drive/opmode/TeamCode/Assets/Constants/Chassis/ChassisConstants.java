package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
/**
 * This class contains all the constants for the chassis
 * All measurements are in inches
 */
public final class ChassisConstants {

  /**
   * The full dimensions of the robot measured from outside to outside
   */
  public static double FullWidth = 13.0;
  public static double FullLength = 15.0;

  /**
   * The dimensions of the robot measured from the center of each wheel to the other
   */
  public static double TrackWidth = 11.0;

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
   * Multiplier to fine tune the linear ticks per inch travled
   */
  public static double LinearMultiplier = 1.0;

  /**
   * The ticks per inch of movement
   */
  public static final double TicksPerInch =
    (TicksPerRev * GearRatio / WheelCircumference) * LinearMultiplier;

  /**
   * Multiplier to fine tune the angular ticks per degree rotated
   */
  public static double AngularMultiplier = 1.0;

  /**
   * The ticks per degree of movement rotation
   */
  public static double TicksPerDegree =
    (TicksPerInch * (TrackWidth / 2) * Math.PI / 180) * AngularMultiplier;

  public static double LinearVelocity = 0.3;
  public static double AngularVelocity = 0.3;
}
