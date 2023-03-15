// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.twilight.swerve.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.twilight.swerve.SwerveDrive;
import frc.twilight.swerve.vectors.DriveVector;
import frc.twilight.swerve.vectors.Position;

public class Swerve extends SubsystemBase {
  private SwerveDrive swerve = new SwerveDrive();

  int time = 0;

  /** Creates a new ExampleSubsystem. */
  public Swerve() {}

  public void setDrive(double x, double y, double rot) {
    swerve.setDrive(new DriveVector(y, x, rot).maxVel());
  }

  public void setDrive(DriveVector vector) {
    swerve.setDrive(vector);
  }

  public void setDrive(DriveVector vector, boolean robotCentric) {
    swerve.setDrive(vector, robotCentric);
  }

  public DriveVector getDrive() {
    return swerve.getDrive();
  }

  @Override
  public void periodic() {
    swerve.updateOdo();
  }

  public void xMode() {
    swerve.xMode();
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void setOdo(double x, double y, double rot) {
    swerve.setOdo(x, y, rot);
  }

  public void setPose2d(Pose2d pose2d) {
    Position position = new Position(pose2d);// converting x to y vise versa

    setOdo(position.getX(), position.getY(), position.getAngle());
  }

  public Position getOdo() {
    return swerve.getOdo();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
