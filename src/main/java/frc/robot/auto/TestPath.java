package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balancing;
import frc.twilight.swerve.subsystems.Swerve;

public class TestPath extends SequentialCommandGroup {
  public TestPath(Swerve swerve) {
    addCommands(new Balancing(swerve));
  }
}
