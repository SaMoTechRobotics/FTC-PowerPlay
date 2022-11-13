package org.firstinspires.ftc.teamcode;

public final class ChassisPosition {
  private double x;
  private double y;
  private double rotation;

  /**
   * Constructor for the chassis position
   * @param x The x position of the robot
   * @param y The y position of the robot
   * @param angle The angle of the robot
   */
  public ChassisPosition(double x, double y, double rotation) {
    this.x = x;
    this.y = y;
    this.rotation = rotation;
  }

  public double getX() {
    return x;
  }

  public void setX(double x) {
    this.x = x;
  }

  public void setY(double y) {
    this.y = y;
  }

  public double getY() {
    return y;
  }

  public double getRotation() {
    return rotation;
  }

  public void setRotation(double rotation) {
      this.rotation = rotation;
  }
}
