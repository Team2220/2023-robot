// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Mobility;
import frc.robot.auto.TestPath;
import frc.robot.auto.leftTwoCubeAuto;
import frc.robot.auto.rightTwoCubeAuto;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Arm.ShoulderPercentOutput;
import frc.robot.commands.Arm.WristPercentOutput;
import frc.robot.commands.Intake.IntakePercentOutput;
// import frc.robot.commands.Leds.RainbowLeds;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.Controller;
import frc.twilight.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.twilight.swerve.commands.ControllerDrive;
import frc.twilight.swerve.commands.ResetGyro;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Swerve m_swerve = new Swerve();
  private final Arm m_arm;
  private final Intake m_intake;
  private final LEDs m_leds;
  private final Limelight m_LimeLight;
  private final DriverTab drivertab = new DriverTab();
  private final CommandChooser autoChooser = new CommandChooser();

  private final Controller m_controller = new Controller(0);
  private final Controller m_secondaryController = new Controller(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    // Starts recording to data log
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    m_arm = new Arm(m_controller);
    m_intake = new Intake();
    m_leds = new LEDs();
    m_LimeLight = new Limelight("limelight");
    m_swerve.setDefaultCommand(
        new ControllerDrive(
            m_swerve,
            () -> m_controller.getLeftX(),
            () -> m_controller.getLeftY(),
            () -> m_controller.getRightX()));
    // Configure the button bindings
    configureButtonBindings();
    // auto stuff
    autoChooser.setDefaultOption(new InstantCommand().withName("Do nothing"));
    autoChooser.addOption(new Mobility(m_swerve));
    autoChooser.addOption(new rightTwoCubeAuto(m_swerve, m_arm, m_intake));
    autoChooser.addOption(new leftTwoCubeAuto(m_swerve, m_arm, m_intake));
    autoChooser.addOption(new TestPath(m_swerve));
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // driver contoller
    new Trigger(() -> m_controller.getButtonPressed(Controller.Button.START))
        .onTrue(new ResetGyro(m_swerve));
    // manipulatror controller

    // Arm & Wrist Joystcks
    new Trigger(() -> (Math.abs(m_secondaryController.getLeftY()) > 0.1))
        .whileTrue(new ShoulderPercentOutput(m_secondaryController::getLeftY, m_arm));
    new Trigger(() -> (Math.abs(m_secondaryController.getRightY()) > 0.1))
        .whileTrue(new WristPercentOutput(m_secondaryController::getRightY, m_arm));
    // Arm States
    new Trigger(() -> (m_secondaryController.getButton(frc.twilight.Controller.Button.A)))
        .onTrue(new SetArmState(ArmStates.MID_CUBE_NODE, m_arm));
    new Trigger(() -> (m_secondaryController.getButton(frc.twilight.Controller.Button.X)))
        .onTrue(new SetArmState(ArmStates.HIGH_CUBE_NODE, m_arm));
    new Trigger(() -> (m_secondaryController.getButton(frc.twilight.Controller.Button.B)))
        .onTrue(new SetArmState(ArmStates.MID_CONE_NODE, m_arm));
    new Trigger(() -> (m_secondaryController.getButton(frc.twilight.Controller.Button.Y)))
        .onTrue(new SetArmState(ArmStates.HIGH_CONE_NODE, m_arm));
    new Trigger(() -> (m_secondaryController.getButton(frc.twilight.Controller.Button.RB)))
        .onTrue(new SetArmState(ArmStates.INTAKE, m_arm));

    // Override limits
    new Trigger(() -> (m_secondaryController.getButton(frc.twilight.Controller.Button.LS)))
        .onTrue(new InstantCommand(() -> m_arm.overrideShoulderSoftLimits(false)))
        .onFalse(new InstantCommand(() -> m_arm.overrideShoulderSoftLimits(true)));

    new Trigger(() -> (m_secondaryController.getButton(frc.twilight.Controller.Button.RS)))
        .onTrue(new InstantCommand(() -> m_arm.overrideWristSoftLimits(false)))
        .onFalse(new InstantCommand(() -> m_arm.overrideWristSoftLimits(true)));

    // Intake Buttons
    new Trigger(() -> m_secondaryController.getButton(frc.twilight.Controller.Button.LB))
        .whileTrue(new IntakePercentOutput(.75, m_intake));
    new Trigger(() -> m_secondaryController.getButton(frc.twilight.Controller.Button.RB))
        .whileTrue(new IntakePercentOutput(-.75, m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
