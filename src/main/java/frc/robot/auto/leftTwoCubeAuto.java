package frc.robot.auto;

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

public class leftTwoCubeAuto extends SequentialCommandGroup {
  public leftTwoCubeAuto(Swerve swerve, Arm arm, Intake intake) {
    addCommands(
        new InstantCommand(() -> swerve.setOdo(3, .5, 0))); // set starting position
    addCommands( // score starting cube
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(-.5, intake).withTimeout(1));
    addCommands( // score cube one
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Position(3.5, 1, 0)),
        new IntakePercentOutput(.5, intake),
        new GoToCommand(swerve, new Position(1, .5, 0)),
        new SetArmState(ArmStates.MID_CUBE_NODE, arm),
        new SetArmState(ArmStates.INTAKE, arm),
        new IntakePercentOutput(-.5, intake).withTimeout(1));
  }
}
