package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class TestPath extends SequentialCommandGroup {
  public TestPath(Swerve swerve) {
    addCommands(
        // new GoToCommand(swerve, new Position(0, 0, 0)),
        // new GoToCommand(swerve, new Position(-3, 0, 0)).setTolerence(0.5, 10),
        // new GoToCommand(swerve, new Position(-3,-3, 0))
    );
  }
}
