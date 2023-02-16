package frc.robot.auto;

import org.littletonrobotics.frc2023.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class Autonomous2 extends SequentialCommandGroup {
  public Autonomous2(Swerve swerve) {
    // right
    addCommands(
        new InstantCommand( // set starting position
            () ->
                swerve.setOdo(1, 1, 0)));
  }
}
