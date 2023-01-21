// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Intake.IntakePercentOutput;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.twilight.Controller;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final Swerve m_swerve;
  private final Arm m_arm;
  private final Intake m_intake;

  private final Controller m_controller = new Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerve = new Swerve(() -> m_controller.getLeftX() * 5, () -> m_controller.getLeftY() * 5, () -> m_controller.getRightX() * 5);
    m_arm = new Arm();
    m_intake = new Intake();

     // Configure the button bindings
     configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new Button(m_controller::getAButton).whenPressed(m_swerve::zeroGyro);
    new Trigger(() -> m_controller.getButton(frc.twilight.Controller.Button.UP))
      .whileTrue(new IntakePercentOutput(0.1, m_intake));
    new Trigger(() -> m_controller.getButton(frc.twilight.Controller.Button.DOWN))
      .whileTrue(new IntakePercentOutput(-0.1, m_intake));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return null;
  }
}
