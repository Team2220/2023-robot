package frc.robot.auto.inProgress;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.IntakePercentOutput;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class UpperScore3FarAsPossibleRedAuto extends SequentialCommandGroup {
  public UpperScore3FarAsPossibleRedAuto(Swerve swerve, Arm arm, Intake intake) {
    addCommands(new InstantCommand(() -> swerve.setOdo(8, 4, 0))); // set starting position
    addCommands( // score starting game piece
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // score game piece 1
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Position(5, 4, 0)),
        new IntakePercentOutput(()->.5, null, intake),
        new GoToCommand(swerve, new Position(8, 3, 0)),
        new SetArmState(ArmStates.HIGH_CUBE_NODE, arm),
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // score game piece 2
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Position(5, 2.5, 0)),
        new IntakePercentOutput(()->.5, null, intake),
        new GoToCommand(swerve, new Position(8, 2.5, 0)),
        new SetArmState(ArmStates.HIGH_CUBE_NODE, arm),
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // score game piece 3
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Position(5, 1.5, 0)),
        new IntakePercentOutput(()->.5, null, intake),
        new GoToCommand(swerve, new Position(8, 2.5, 0)),
        new SetArmState(ArmStates.HIGH_CUBE_NODE, arm),
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // score game piece 4
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Position(5, 1, 0)),
        new IntakePercentOutput(()->.5, null, intake),
        new GoToCommand(swerve, new Position(8, 3, 0)),
        new SetArmState(ArmStates.HIGH_CUBE_NODE, arm),
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
  }
}
