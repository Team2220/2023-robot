package frc.robot.auto.inProgress;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class MidScore1BalBlueAuto extends SequentialCommandGroup {
  public MidScore1BalBlueAuto(Swerve swerve, Arm arm, Intake intake) {
    addCommands(new InstantCommand(() -> swerve.setOdo(.5, 1.5, 0))); // set starting position
    addCommands( // score starting game piece
        new SetArmState(ArmStates.INTAKE, arm),
        new AutoIntake(.5, intake).withTimeout(1),
        new SetArmState(ArmStates.INTAKE, arm),
        new AutoIntake(.5, intake).withTimeout(1));
    addCommands( // balance
        new GoToCommand(swerve, new Position(2, 2, 0)), 
        new Balancing(swerve));
  }
}
