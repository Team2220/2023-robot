// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.twilight.swerve.commands;

import frc.twilight.helpfulThings.Angles;
import frc.twilight.swerve.config.GeneralConfig;
import frc.twilight.swerve.config.PIDconfig;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.DriveVector;
import frc.twilight.swerve.vectors.Position;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

  private boolean xDone = false;
  private boolean yDone = false;
  private boolean rotDone = false;

  private double movTol = 0.05;
  private double rotTol = 2.5;

  private boolean stopAtEnd = true;

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
  }

  public GoToCommand(Swerve subsystem, Pose2d goal) {
    this(subsystem, new Position(goal));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Position currentPos = m_subsystem.getOdo();
    DriveVector currentVec = m_subsystem.getDrive();
    setpointX = new TrapezoidProfile.State(currentPos.getX(), currentVec.getStr());
    setpointY = new TrapezoidProfile.State(currentPos.getY(), currentVec.getFwd());
    setpointRot = new TrapezoidProfile.State(currentPos.getAngle(), currentVec.getRcw());

    double angle = currentPos.getAngle();
    double angleNew = goalRot.position;

    goalRot.position = Angles.flipAround(angle, angleNew);

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

    m_subsystem.setDrive(new DriveVector(yVel, xVel, rotVel).maxVel());

    if (movTol > 0) {
      xDone = Math.abs(currentPos.getX() - goalX.position) < movTol;
      yDone = Math.abs(currentPos.getY() - goalY.position) < movTol;
    } else {
      xDone = true;
      yDone = true;
    }

    if (rotTol > 0)
      rotDone = Math.abs(currentPos.getAngle() - goalRot.position) < rotTol;
    else
      rotDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopAtEnd)
      m_subsystem.setDrive(new DriveVector(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xDone && yDone && rotDone;
  }

  public GoToCommand setTolerence(double mov, double rot) {
    movTol = mov;
    rotTol = rot;

    return this;
  }

  public GoToCommand endPos(double x, double y, double rot) {
    goalX.position = x;
    goalY.position = y;
    goalRot.position = rot;

    initialize();

    return this;
  }

  public GoToCommand endVel(double xVel, double yVel, double rotVel) {
    goalX.velocity = xVel;
    goalY.velocity = yVel;
    goalRot.velocity = rotVel;

    initialize();

    return this;
  }

  public GoToCommand stopAtEnd(boolean stop) {
    stopAtEnd = stop;

    return this;
  }
}
