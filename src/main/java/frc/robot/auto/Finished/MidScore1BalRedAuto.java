// unfinished

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.IntakePercentOutput;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Intake;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class MidScore1BalRedAuto extends SequentialCommandGroup {
  public MidScore1BalRedAuto(Swerve swerve, Arm arm, Intake intake) {
    addCommands(new InstantCommand(() -> swerve.setOdo(1.8453771028, 1.071499, 180))); // set starting position
    addCommands( // score starting game piece
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // score game piece 1
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Position(7.070376, 0.919099, 0)),
        new IntakePercentOutput(() -> .5, null, intake),withTimeout(1),
        new GoToCommand(swerve, new Position(1.8453771028, 1.071499, 180)),
        new SetArmState(ArmStates.HIGH_CUBE_NODE, arm),
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(()->-.5, null, intake).withTimeout(1));
    addCommands( // balance
        new GoToCommand(swerve, new Position(3.279775, 2.743581, 0)), 
        new Balancing(swerve));
  }
}
