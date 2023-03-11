// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Square;
import frc.robot.auto.Finished.BlueCornerMobility;
import frc.robot.auto.Finished.MidScore1BalBlueAutoL;
import frc.robot.auto.Finished.MidScore1BalBlueAutoR;
import frc.robot.auto.Finished.RedCornerMobility;
import frc.robot.auto.TestPath;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Arm.ArmPercentOutput;
import frc.robot.commands.Intake.IntakePercentOutput;
import frc.robot.commands.Leds.SetLedsStates;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.LEDs.DesieredState;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.twilight.swerve.commands.ControllerDrive;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.twilight.Controller.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /*
     * Swerve Subsystem
     */
    public static final Swerve m_swerve = new Swerve();

    /*
     * Arm Subsystem
     */
    public static final Arm m_arm = new Arm();

    /*
     * Intake Subsystem
     */
    public static final Intake m_intake = new Intake();;

    /*
     * LEDs not used rn
     */
    @SuppressWarnings("unused")
    public static final LEDs m_leds = new LEDs();

    /*
     * Drivertab?
     */
    @SuppressWarnings("unused")
    public static final DriverTab drivertab = new DriverTab();

    /*
     * Autochooser
     */
    public static final CommandChooser autoChooser = new CommandChooser();

    /*
     * Driver Controller
     */
    public static final Controller m_controller = new Controller(0);

    /*
     * Manipulator Controller
     */
    public static final Controller m_secondaryController = new Controller(1);

    /*
     * Default Drive Command
     */
    public static final ControllerDrive m_ControllerDrive = new ControllerDrive(
            m_swerve,
            () -> m_controller.getLeftX(),
            () -> m_controller.getLeftY(),
            () -> m_controller.getRightX());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Stop logging for missing joysticks
        DriverStation.silenceJoystickConnectionWarning(true);

        // Starts recording to data log
        DataLogManager.start();

        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());

        // Sets the default command for the drivetrain
        m_swerve.setDefaultCommand(m_ControllerDrive);

        // Sets the default command for the arm
        // m_arm.setDefaultCommand(
        // new ArmPercentOutput(
        // m_secondaryController::getRightY, m_secondaryController::getLeftY, m_arm));

        // Sets the default command for the intake
        m_intake.setDefaultCommand(
                new IntakePercentOutput(
                        m_secondaryController::getLeftTrigger,
                        m_secondaryController::getRightTrigger,
                        m_intake));

        // Configure the button bindings
        configureButtonBindings();

        // auto stuff
        autoChooser.setDefaultOption(new InstantCommand().withName("Do nothing"));
        autoChooser.addOption(new Square(m_swerve));
        // autoChooser.addOption(new rightTwoCubeAuto(m_swerve, m_arm, m_intake));
        // autoChooser.addOption(new leftTwoCubeAuto(m_swerve, m_arm, m_intake));
        autoChooser.addOption(new TestPath(m_swerve, m_arm, m_intake));
        // autoChooser.addOption(new NewPath(m_swerve));
        autoChooser.addOption(new MidScore1BalBlueAutoR(m_swerve, m_arm, m_intake));
        autoChooser.addOption(new BlueCornerMobility(m_swerve, m_intake));
        autoChooser.addOption(new RedCornerMobility(m_swerve));

        SmartDashboard.putData(autoChooser.getSendableChooser());

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Map Driver Controller Buttons
        ControllerLayout.mapDriverController(m_controller);
        new Trigger(() -> m_controller.getButtonPressed(Controller.Button.LB))
                .onTrue(new SetLedsStates(DesieredState.WANT_CONE, m_leds));

        new Trigger(() -> m_controller.getButtonPressed(Controller.Button.RB))
                .onTrue(new SetLedsStates(DesieredState.WANT_CUBE, m_leds));
        // Map Manipulator Controller Buttons
        ControllerLayout.mapManipulatorController(m_secondaryController);

        // Manipulatror controller
        new Trigger(() -> m_secondaryController.getButtonPressed(Controller.Button.BACK))
                .onTrue(new InstantCommand(() -> DataLogManager.log("Manipulator Problem")));

        new Trigger(() -> m_secondaryController.getButtonPressed(Controller.Button.START))
                .onTrue(new InstantCommand(() -> {
                    m_arm.setWristToReferenceAngle();
                    m_arm.setShoulderToReferenceAngle();
                }));
        new Trigger(
                () -> {
                    boolean left = Math.abs(m_secondaryController.getLeftY()) > 0.1;
                    boolean right = Math.abs(m_secondaryController.getRightY()) > 0.1;
                    return left || right;
                })
                .whileTrue(
                        new ArmPercentOutput(
                                m_secondaryController::getRightY, m_secondaryController::getLeftY, m_arm));

        new Trigger(
                () -> {
                    boolean rightX = Math.abs(m_controller.getRightX()) > 0.1;
                    boolean enabled = m_ControllerDrive.isScheduled();
                    return rightX && !enabled;
                }).whileTrue(m_ControllerDrive);

        // Arm States
        new Trigger(() -> (m_secondaryController.getButton(Button.RB)))
                .onTrue(new SetArmState(ArmStates.INTAKE, m_arm));
        new Trigger(() -> (m_secondaryController.getButton(Button.UP)))
                .onTrue(new SetArmState(ArmStates.TRANSIT, m_arm));

        new Trigger(() -> (m_secondaryController.getButton(Button.X)))
                .whileTrue(new SetArmState(ArmStates.SINGLE_LOADING_STATION, m_arm));

        new Trigger(() -> (m_secondaryController.getButton(Button.B)))
                .whileTrue(new SetArmState(ArmStates.DOUBLE_LOADING_STATION, m_arm));

        new Trigger(() -> (m_secondaryController.getButton(Button.Y)))
                .and(() -> (!m_secondaryController.getButton(Button.DOWN)))
                .whileTrue(new SetArmState(ArmStates.HIGH_CONE_NODE, m_arm));
        new Trigger(() -> (m_secondaryController.getButton(Button.Y)))
                .and(() -> (m_secondaryController.getButton(Button.DOWN)))
                .whileTrue(new SetArmState(ArmStates.HIGH_CUBE_NODE, m_arm));

        new Trigger(() -> (m_secondaryController.getButton(Button.A)))
                .and(() -> (!m_secondaryController.getButton(Button.DOWN)))
                .whileTrue(new SetArmState(ArmStates.MID_CONE_NODE, m_arm));
        new Trigger(() -> (m_secondaryController.getButton(Button.A)))
                .and(() -> (m_secondaryController.getButton(Button.DOWN)))
                .whileTrue(new SetArmState(ArmStates.MID_CUBE_NODE, m_arm));

        // Override limits
        new Trigger(() -> (m_secondaryController.getButton(Button.LS)))
                .onTrue(new InstantCommand(() -> m_arm.overrideShoulderSoftLimits(false)))
                .onFalse(new InstantCommand(() -> m_arm.overrideShoulderSoftLimits(true)));

        new Trigger(() -> (m_secondaryController.getButton(Button.RS)))
                .onTrue(new InstantCommand(() -> m_arm.overrideWristSoftLimits(false)))
                .onFalse(new InstantCommand(() -> m_arm.overrideWristSoftLimits(true)));
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
