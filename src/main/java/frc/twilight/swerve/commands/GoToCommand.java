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

import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GoToCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Swerve m_subsystem;

  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      3, GeneralConfig.DT_MAX_ACCEL.getValue());
  private TrapezoidProfile.Constraints constraintsRot = new TrapezoidProfile.Constraints(
      360, GeneralConfig.DT_MAX_ROT_ACCEL.getValue());

  private final Position goal;

  private TrapezoidProfile.State goalX = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalY = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalRot = new TrapezoidProfile.State();

  private TrapezoidProfile.State setpointX = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointY = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointRot = new TrapezoidProfile.State();

  private TrapezoidProfile profileX;
  private TrapezoidProfile profileY;
  private TrapezoidProfile profileRot;

  private PIDController pidX = new PIDController(
      PIDconfig.DT_AUTO_P.getValue(),
      PIDconfig.DT_AUTO_I.getValue(),
      PIDconfig.DT_AUTO_D.getValue());
  private PIDController pidY = new PIDController(
      PIDconfig.DT_AUTO_P.getValue(),
      PIDconfig.DT_AUTO_I.getValue(),
      PIDconfig.DT_AUTO_D.getValue());
  private PIDController pidRot = new PIDController(
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

  private static double drivingX = 0;
  private static double drivingY = 0;
  private static double drivingRot = 0;

  private static double outputX = 0;
  private static double outputY = 0;
  private static double outputRot = 0;

  private static boolean shuffled = false;

  /**
   * Creates a new GoToCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToCommand(Swerve subsystem, Position goal, double vel, double rotVel) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    this.goal = goal;

    constraints = new TrapezoidProfile.Constraints(
        vel, GeneralConfig.DT_MAX_ACCEL.getValue());
    constraintsRot = new TrapezoidProfile.Constraints(
        rotVel, GeneralConfig.DT_MAX_ROT_ACCEL.getValue());

    if (!shuffled) {
      Shuffleboard.getTab("Swerve").addDouble("Current Goto X", () -> {
        return drivingX;
      });
      Shuffleboard.getTab("Swerve").addDouble("Current Goto Y", () -> {
        return drivingY;
      });
      Shuffleboard.getTab("Swerve").addDouble("Current Goto Rot", () -> {
        return drivingRot;
      });

      Shuffleboard.getTab("Swerve").addDouble("Current Driving X", () -> {
        return outputX;
      });
      Shuffleboard.getTab("Swerve").addDouble("Current Driving Y", () -> {
        return outputY;
      });
      Shuffleboard.getTab("Swerve").addDouble("Current Driving Rot", () -> {
        return outputRot;
      });

      Shuffleboard.getTab("DEBUG").addBoolean("ShouldFlip", () -> AllianceFlipUtil.shouldFlip());

      shuffled = true;
    }
  }

  public GoToCommand(Swerve subsystem, Position position) {
    this(subsystem, position, 3, 360);
  }

  public GoToCommand(Swerve subsystem, Pose2d goal) {
    this(subsystem, new Position(goal));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Position goal2 = AllianceFlipUtil.apply(new Position(goal.getX(), goal.getY(), goal.getAngle()));

    goalX = new TrapezoidProfile.State(goal2.getX(), 0);
    goalY = new TrapezoidProfile.State(goal2.getY(), 0);
    goalRot = new TrapezoidProfile.State(goal2.getAngle(), 0);

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

    DataLogManager.log("Running GoTo Command - \n" +
        "\tStarting Point: " + setpointX.position + ", " + setpointY.position + ", " + setpointRot.position + "\n" +
        "\tEnd Point: " + goalX.position + ", " + goalY.position + ", " + goalRot.position + "\n");

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

    drivingX = xPos;
    drivingY = yPos;
    drivingRot = rotPos;

    outputX = xVel;
    outputY = yVel;
    outputRot = rotVel;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("End GoToCommand");
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
