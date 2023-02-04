// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.twilight.swerve.commands;

import frc.twilight.swerve.config.GeneralConfig;
import frc.twilight.swerve.config.PIDconfig;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.DriveVector;
import frc.twilight.swerve.vectors.Position;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GoToCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_subsystem;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          GeneralConfig.DT_MAX_VEL.getValue(), GeneralConfig.DT_MAX_ACCEL.getValue());
  private final TrapezoidProfile.Constraints constraintsRot =
      new TrapezoidProfile.Constraints(
          GeneralConfig.DT_MAX_ROT_VEL.getValue(), GeneralConfig.DT_MAX_ROT_ACCEL.getValue());

  private TrapezoidProfile.State goalX = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalY = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalRot = new TrapezoidProfile.State();

  private TrapezoidProfile.State setpointX = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointY = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointRot = new TrapezoidProfile.State();

  private TrapezoidProfile profileX;
  private TrapezoidProfile profileY;
  private TrapezoidProfile profileRot;

  private PIDController pidX =
      new PIDController(
          PIDconfig.DT_AUTO_P.getValue(),
          PIDconfig.DT_AUTO_I.getValue(),
          PIDconfig.DT_AUTO_D.getValue());
  private PIDController pidY =
      new PIDController(
          PIDconfig.DT_AUTO_P.getValue(),
          PIDconfig.DT_AUTO_I.getValue(),
          PIDconfig.DT_AUTO_D.getValue());
  private PIDController pidRot =
      new PIDController(
          PIDconfig.DT_AUTO_ROT_P.getValue(),
          PIDconfig.DT_AUTO_ROT_I.getValue(),
          PIDconfig.DT_AUTO_ROT_D.getValue());

  private double kDt = 0;

  /**
   * Creates a new GoToCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToCommand(Swerve subsystem, Position goal) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    goalX = new TrapezoidProfile.State(goal.getX(), 0);
    goalY = new TrapezoidProfile.State(goal.getY(), 0);
    goalRot = new TrapezoidProfile.State(goal.getAngle(), 0);

    pidX.setTolerance(0.05);
    pidY.setTolerance(0.05);
    pidRot.setTolerance(5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Position currentPos = m_subsystem.getOdo();
    DriveVector currentVec = m_subsystem.getDrive();
    setpointX = new TrapezoidProfile.State(currentPos.getX(), currentVec.getStr());
    setpointY = new TrapezoidProfile.State(currentPos.getY(), currentVec.getFwd());
    setpointRot = new TrapezoidProfile.State(currentPos.getAngle(), currentVec.getRcw());

    profileX = new TrapezoidProfile(constraints, goalX, setpointX);
    profileY = new TrapezoidProfile(constraints, goalY, setpointY);
    profileRot = new TrapezoidProfile(constraintsRot, goalRot, setpointRot);

    kDt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kDt += 0.02;

    double xPos = profileX.calculate(kDt).position;
    double yPos = profileY.calculate(kDt).position;
    double rotPos = profileRot.calculate(kDt).position;

    // Update PID
    pidX.setPID(
        PIDconfig.DT_AUTO_P.getValue(),
        PIDconfig.DT_AUTO_I.getValue(),
        PIDconfig.DT_AUTO_D.getValue());
    pidY.setPID(
        PIDconfig.DT_AUTO_P.getValue(),
        PIDconfig.DT_AUTO_I.getValue(),
        PIDconfig.DT_AUTO_D.getValue());
    pidRot.setPID(
        PIDconfig.DT_AUTO_ROT_P.getValue(),
        PIDconfig.DT_AUTO_ROT_I.getValue(),
        PIDconfig.DT_AUTO_ROT_D.getValue());

    Position currentPos = m_subsystem.getOdo();

    double xVel = pidX.calculate(currentPos.getX(), xPos);
    double yVel = pidY.calculate(currentPos.getY(), yPos);
    double rotVel = pidRot.calculate(currentPos.getAngle(), rotPos);

    m_subsystem.setDrive(
        new DriveVector(yVel, xVel, rotVel).zeroDirection(currentPos.getAngle()).maxVel());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setDrive(new DriveVector(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidX.atSetpoint()
        && pidY.atSetpoint()
        && pidRot.atSetpoint()
        && profileX.isFinished(kDt)
        && profileY.isFinished(kDt)
        && profileRot.isFinished(kDt);
    // return false;
  }
}