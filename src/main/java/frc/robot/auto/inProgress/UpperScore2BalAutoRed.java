// unfinished

package frc.robot.auto.inProgress;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.IntakePercentOutput;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class UpperScore2BalAutoRed extends SequentialCommandGroup {
  public UpperScore2BalAutoRed(Swerve swerve, Arm arm, Intake intake) {
    addCommands(new InstantCommand(() -> swerve.setOdo(.5, 3, 0))); // set starting position
    addCommands( // score starting game piece 
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // score first game piece
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Position(4, 4, 0)),
        new IntakePercentOutput(()->.5, null, intake).withTimeout(1),
        new GoToCommand(swerve, new Position(.5, 3, 0)),
        new SetArmState(ArmStates.HIGH_CUBE_NODE, arm),
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // score second game piece
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Position(4, 2.5, 0)),
        new IntakePercentOutput(()->.5, null, intake).withTimeout(1),
        new GoToCommand(swerve, new Position(.5, 2.5, 0)),
        new SetArmState(ArmStates.HIGH_CUBE_NODE, arm),
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // balance
        new GoToCommand(swerve, new Position(2, 2, 0)), 
        new Balancing(swerve));
  }
}
