package frc.twilight.swerve.vectors;

public class WheelVector {
  private double velocity;
  private double angle;

  public WheelVector(double velocity, double angle) {
    this.velocity = velocity;
    this.angle = angle;
  }

  public double getVelocity() {
    return velocity;
  }

  public double getAngle() {
    return angle;
  }

  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  public void setAngle(double angle) {
    this.angle = angle;
  }
}
