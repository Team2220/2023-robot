// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.twilight.swerve.commands;

import frc.twilight.swerve.config.GeneralConfig;
import frc.twilight.swerve.config.PIDconfig;
import frc.twilight.swerve.devices.Gyro;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.DriveVector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ControllerDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_subsystem;

  public DoubleSupplier fwd;
  public DoubleSupplier str;
  public DoubleSupplier rot;

  private boolean snapDrive = false;
  private double snapRot = 0;
  private PIDController rotpid = new PIDController(PIDconfig.DT_AUTO_ROT_P.getValue(), PIDconfig.DT_AUTO_ROT_I.getValue(), PIDconfig.DT_AUTO_ROT_D.getValue());
  
  private double maxVelMov = GeneralConfig.DT_MAX_VEL.getValue();
  private double maxVelRot = GeneralConfig.DT_MAX_ROT_VEL.getValue();

  /**
   * Creates a new GoToCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ControllerDrive(Swerve subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rcw) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    fwd = y;
    str = x;
    rot = rcw;

    rotpid.enableContinuousInput(0, 360);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveVector.resetAccel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = rot.getAsDouble() * maxVelRot;
    
    if (snapDrive) {
      rotation = rotpid.calculate(Gyro.getAngle() % 360, snapRot);
    }

    m_subsystem.setDrive(
        new DriveVector(
                fwd.getAsDouble() * maxVelMov,
                str.getAsDouble() * maxVelMov,
                rotation)
            .maxVel(maxVelMov, maxVelRot)
            .maxAccel());
  }

  public void snapDrive(double rot) {
    snapRot = rot;
    snapDrive = true;
  }

  public void stopSnap() {
    snapDrive = false;
  }

  public void setMaxVel(double mov, double rot) {
    maxVelMov = mov;
    maxVelRot = rot;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_subsystem.setDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
