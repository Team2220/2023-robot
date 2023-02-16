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
    addCommands(
        new InstantCommand( // set starting position
            () ->
                swerve.setPose2d(
                    new Pose2d(
                        FieldConstants.Community.chargingStationCorners[0],
                        Rotation2d.fromDegrees(90)))));
    addCommands(
        new GoToCommand( // move away from charging station
            swerve, new Position(2, 0, 0)));
    addCommands( // crossing line
        new GoToCommand(swerve, new Position(4, 0, 0)));
    // picking cube fucntion goes here verify arm code works
    addCommands( // carrying cube
        new GoToCommand(swerve, new Position(4, 4, 0)));
    addCommands( // returning
        new GoToCommand(swerve, new Position(1, 1, 0)));
  }
}