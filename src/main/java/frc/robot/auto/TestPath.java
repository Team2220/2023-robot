package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.commands.SequentialMove;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class TestPath extends SequentialCommandGroup {
  public TestPath(Swerve swerve) {
    addCommands(
        new SequentialMove(
            swerve, new Position(0, 0, 0), new Position(0, 3, 0), new Position(3, 3, 0)));
  }
}
