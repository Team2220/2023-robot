// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.EventLoops;
import frc.twilight.PDHLogPowerFaults;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   * 
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    addPeriodic(EventLoops.oncePerSec::poll, 1); // for temp logger
    addPeriodic(EventLoops.oncePerMin::poll, 60); // for stall logger
    SmartDashboard.putData(CommandScheduler.getInstance());
    Shuffleboard.getTab("infrastructure").addNumber("Voltage", () -> RobotController.getBatteryVoltage());
    Shuffleboard.getTab("infrastructure").addNumber("Value", () -> RobotController.getBatteryVoltage());
    Shuffleboard.getTab("infrastructure").addNumber("CanUsage",
        () -> RobotController.getCANStatus().percentBusUtilization);
    PowerDistribution pdh = new PowerDistribution();
    PDHLogPowerFaults.setPdh(pdh);
    Shuffleboard.getTab("infrastructure").addNumber("chan0", () -> pdh.getCurrent(0));
    Shuffleboard.getTab("infrastructure").addNumber("chan1", () -> pdh.getCurrent(1));
    Shuffleboard.getTab("infrastructure").addNumber("chan2", () -> pdh.getCurrent(2));
    Shuffleboard.getTab("infrastructure").addNumber("chan3", () -> pdh.getCurrent(3));
    Shuffleboard.getTab("infrastructure").addNumber("chan4", () -> pdh.getCurrent(4));
    Shuffleboard.getTab("infrastructure").addNumber("chan5", () -> pdh.getCurrent(5));
    Shuffleboard.getTab("infrastructure").addNumber("chan6", () -> pdh.getCurrent(6));
    Shuffleboard.getTab("infrastructure").addNumber("chan7", () -> pdh.getCurrent(7));
    Shuffleboard.getTab("infrastructure").addNumber("chan8", () -> pdh.getCurrent(8));
    Shuffleboard.getTab("infrastructure").addNumber("chan9", () -> pdh.getCurrent(9));
    Shuffleboard.getTab("infrastructure").addNumber("chan10", () -> pdh.getCurrent(10));
    Shuffleboard.getTab("infrastructure").addNumber("chan11", () -> pdh.getCurrent(11));
    Shuffleboard.getTab("infrastructure").addNumber("chan12", () -> pdh.getCurrent(12));
    Shuffleboard.getTab("infrastructure").addNumber("chan13", () -> pdh.getCurrent(13));
    Shuffleboard.getTab("infrastructure").addNumber("chan14", () -> pdh.getCurrent(14));
    Shuffleboard.getTab("infrastructure").addNumber("", () -> pdh.getCurrent(15));
    Shuffleboard.getTab("infrastructure").addNumber("Total Current", () -> pdh.getTotalCurrent());

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
