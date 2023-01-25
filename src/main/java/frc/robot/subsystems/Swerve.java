// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.twilight.swerve.SwerveDrive;
import frc.twilight.swerve.vectors.DriveVector;


public class Swerve extends SubsystemBase {

  
  private SwerveDrive swerve = new SwerveDrive();

  private DoubleSupplier x_axis;
  private DoubleSupplier y_axis;

  private DoubleSupplier rotation;

  int time = 0;

  /** Creates a new Swerve. */
  public Swerve(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    x_axis = x;
    y_axis = y;
    rotation = rot;
  }

  @Override
  public void periodic() {
    DriveVector out = new DriveVector(-y_axis.getAsDouble(), -x_axis.getAsDouble(), rotation.getAsDouble());

    swerve.setDrive(out);

    swerve.updateOdo();
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
