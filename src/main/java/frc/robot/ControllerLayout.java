package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.Controller.Button;
import frc.twilight.swerve.commands.ResetGyro;

public class ControllerLayout {
    public final static Map<Button, Command> DRIVER = Map.ofEntries(
        Map.entry(Button.A, null),
        Map.entry(Button.B, null),
        Map.entry(Button.X, null),
        Map.entry(Button.Y, null),

        // Bumpers
        Map.entry(Button.LB, null),
        Map.entry(Button.RB, null),
        
        // Joystick buttons
        Map.entry(Button.LS, null),
        Map.entry(Button.RS, null),

        // Start (right) button
        Map.entry(Button.START, new ResetGyro(RobotContainer.m_swerve)),

        // Back (left) button
        Map.entry(Button.BACK, new InstantCommand(() -> DataLogManager.log("Driver Problem"))),

        // POV buttons
        Map.entry(Button.UP, new InstantCommand(() -> RobotContainer.m_ControllerDrive.snapDrive(0))),
        Map.entry(Button.DOWN, new InstantCommand(() -> RobotContainer.m_ControllerDrive.snapDrive(180))),
        Map.entry(Button.LEFT, new InstantCommand(() -> RobotContainer.m_ControllerDrive.snapDrive(90))),
        Map.entry(Button.RIGHT, new InstantCommand(() -> RobotContainer.m_ControllerDrive.snapDrive(270)))
    );

    public final static Map<Button, Command> MANIPULATOR = Map.ofEntries(
        Map.entry(Button.A, (new SetArmState(ArmStates.MID_CUBE_NODE, RobotContainer.m_arm))),
        Map.entry(Button.B, (new SetArmState(ArmStates.MID_CONE_NODE, RobotContainer.m_arm))),
        Map.entry(Button.X, (new SetArmState(ArmStates.HIGH_CUBE_NODE, RobotContainer.m_arm))),
        Map.entry(Button.Y, (new SetArmState(ArmStates.HIGH_CONE_NODE, RobotContainer.m_arm))),

        // Bumpers
        Map.entry(Button.LB, null),
        Map.entry(Button.RB, (new SetArmState(ArmStates.INTAKE, RobotContainer.m_arm))),
        
        // Joystick buttons
        Map.entry(Button.LS, (new InstantCommand(() -> RobotContainer.m_arm.overrideShoulderSoftLimits(false)))),
        Map.entry(Button.RS, (new InstantCommand(() -> RobotContainer.m_arm.overrideWristSoftLimits(false)))),

        // Start (right) button
        Map.entry(Button.START, (new InstantCommand(() -> {
            RobotContainer.m_arm.setWristToReferenceAngle();
            RobotContainer.m_arm.setShoulderToReferenceAngle();
        }))),

        // Back (left) button
        Map.entry(Button.BACK, (new InstantCommand(() -> DataLogManager.log("Manipulator Problem")))),

        // POV buttons
        Map.entry(Button.UP,(new SetArmState(ArmStates.LOADING_STATION_CONE, RobotContainer.m_arm))),
        Map.entry(Button.DOWN, (new SetArmState(ArmStates.TRANSIT, RobotContainer.m_arm))),
        Map.entry(Button.LEFT, (new SetArmState(ArmStates.LOADING_STATION_CUBE, RobotContainer.m_arm))),
        Map.entry(Button.RIGHT, null)
    );
}
