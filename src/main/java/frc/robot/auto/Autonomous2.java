package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.subsystems.Swerve;

public class Autonomous2 extends SequentialCommandGroup {
  public Autonomous2(Swerve swerve) {
    // right
    addCommands(
        new InstantCommand( // set starting position
            () -> swerve.setOdo(1, 1, 0)));
  }
}
