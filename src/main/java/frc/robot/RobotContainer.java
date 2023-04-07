// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.TestPath;
import frc.robot.auto.Finished.WireBal;
import frc.robot.auto.Finished.CenterBal;
import frc.robot.auto.Finished.JustScoreTheConeHigh;
import frc.robot.auto.Finished.JustScoreTheConeLow;
import frc.robot.auto.Finished.JustScoreTheConeMid;
import frc.robot.auto.Finished.JustScoreTheCubeHigh;
import frc.robot.auto.Finished.JustScoreTheCubeLow;
import frc.robot.auto.Finished.JustScoreTheCubeMid;
import frc.robot.auto.Finished.MidScore1BalBlueAutoL;
import frc.robot.auto.Finished.MidScore1BlueAutoR;
import frc.robot.auto.Finished.MobilityL;
import frc.robot.auto.Finished.MobilityR;
import frc.robot.auto.Finished.NonWireBal;
import frc.robot.auto.Finished.ScoreAndGetC;
import frc.robot.auto.Finished.ScoreGetAndScoreC;
import frc.robot.auto.Finished.V2MidScoreL;
import frc.robot.auto.Finished.V2MidScoreR;
import frc.robot.auto.Finished.V3CenterBal;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.CommandChooser;
import frc.robot.commands.Arm.ArmPercentOutput;
import frc.robot.commands.Intake.IntakePercentOutput;
import frc.robot.commands.Leds.SetLedsStates;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.LEDs.DesiredState;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.twilight.swerve.commands.ControllerDrive;
import frc.twilight.swerve.config.GeneralConfig;
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
        public final LEDs m_leds;

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
                m_leds = new LEDs(() -> m_intake.isStalled());
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
                // autoChooser.addOption(new Square(m_swerve));
                // autoChooser.addOption(new rightTwoCubeAuto(m_swerve, m_arm, m_intake));
                // autoChooser.addOption(new leftTwoCubeAuto(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new TestPath(m_swerve, m_arm, m_intake));
                // autoChooser.addOption(new NewPath(m_swerve));
                autoChooser.addOption(new MidScore1BlueAutoR(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new WireBal(m_swerve, m_intake, m_arm));
                autoChooser.addOption(new NonWireBal(m_swerve, m_intake, m_arm));
                autoChooser.addOption(new MidScore1BalBlueAutoL(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new MobilityL(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new MobilityR(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new JustScoreTheCubeHigh(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new JustScoreTheCubeMid(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new JustScoreTheCubeLow(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new JustScoreTheConeHigh(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new JustScoreTheConeMid(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new JustScoreTheConeLow(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new ScoreAndGetC(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new ScoreGetAndScoreC(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new CenterBal(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new SetArmState(ArmStates.TRANSIT, m_arm));
                autoChooser.addOption(new V2MidScoreR(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new V2MidScoreL(m_swerve, m_arm, m_intake));
                autoChooser.addOption(new V3CenterBal(m_swerve, m_arm, m_intake));

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
                new Trigger(() -> m_controller.getButton(Controller.Button.LB))
                                .onTrue(new SetLedsStates(DesiredState.WANT_CONE, m_leds))
                                .onFalse(new SetLedsStates(DesiredState.OFF, m_leds));

                new Trigger(() -> m_controller.getButton(Controller.Button.RB))
                                .onTrue(new SetLedsStates(DesiredState.WANT_CUBE, m_leds))
                                .onFalse(new SetLedsStates(DesiredState.OFF, m_leds));

                new Trigger(() -> Math.abs(m_controller.getRightX()) > 0.1)
                                .onTrue(new InstantCommand(() -> m_ControllerDrive.stopSnap()));

                new Trigger(() -> m_controller.getLeftTrigger() > 0.4)
                                .onTrue(new InstantCommand(() -> m_ControllerDrive.setMaxVel(1, 90)))
                                .onFalse(new InstantCommand(() -> m_ControllerDrive.setMaxVel(
                                                GeneralConfig.DT_MAX_VEL.getValue(),
                                                GeneralConfig.DT_MAX_ROT_VEL.getValue())));

                // Map Manipulator Controller Buttons
                ControllerLayout.mapManipulatorController(m_secondaryController);

                new Trigger(() -> m_secondaryController.getButton(Controller.Button.LEFT))
                                .onTrue(new SetLedsStates(DesiredState.WANT_CONE, m_leds))
                                .onFalse(new SetLedsStates(DesiredState.OFF, m_leds));

                new Trigger(() -> m_secondaryController.getButton(Controller.Button.RIGHT))
                                .onTrue(new SetLedsStates(DesiredState.WANT_CUBE, m_leds))
                                .onFalse(new SetLedsStates(DesiredState.OFF, m_leds));

                // Manipulator controller
                new Trigger(() -> m_secondaryController.getButton(Controller.Button.BACK))
                                .onTrue(new InstantCommand(() -> DataLogManager.log("Manipulator Problem")));

                new Trigger(
                                () -> {
                                        boolean left = Math.abs(m_secondaryController.getLeftY()) > 0.1;
                                        boolean right = Math.abs(m_secondaryController.getRightY()) > 0.1;
                                        return left || right;
                                })
                                .whileTrue(
                                                new ArmPercentOutput(
                                                                m_secondaryController::getRightY,
                                                                m_secondaryController::getLeftY, m_arm));

                new Trigger(
                                () -> {
                                        boolean rightX = Math.abs(m_controller.getRightX()) > 0.1;
                                        boolean enabled = m_ControllerDrive.isScheduled();
                                        return rightX && !enabled;
                                }).whileTrue(m_ControllerDrive);

                // Reset encoder arm
                new Trigger(() -> (m_secondaryController.getButton(Button.START)))
                                .onTrue(new InstantCommand(() -> m_arm.setWristFromAbsEncoder()))
                                .onTrue(new InstantCommand(() -> m_arm.setShoulderFromAbsEncoder()));

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
