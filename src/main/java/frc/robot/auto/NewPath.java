// Half of robot: 16.291382




package frc.robot.auto;

import org.littletonrobotics.frc2023.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;

public class NewPath extends SequentialCommandGroup {
  public NewPath(Swerve swerve) {
    addCommands(
        new InstantCommand(
            () ->
                swerve.setPose2d(
                    new Pose2d(
                        FieldConstants.Community.chargingStationCorners[0],
                        Rotation2d.fromDegrees(90)))));
    addCommands(
        new GoToCommand(
            swerve,
            new Pose2d(
                FieldConstants.Community.chargingStationCorners[2].plus(new Translation2d(0.4138011028, 0)), Rotation2d.fromDegrees(0))));

  } 
}

