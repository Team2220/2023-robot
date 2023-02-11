package frc.robot.auto;

import org.littletonrobotics.frc2023.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;

public class Mobility extends SequentialCommandGroup {
  public Mobility(Swerve swerve) {
    addCommands(
        new InstantCommand(
            () ->
                swerve.setPose2d(
                  new Pose2d(FieldConstants.Community.chargingStationCorners[0], Rotation2d.fromDegrees(90)))
                ));
    addCommands(
        new GoToCommand(
            swerve,
            new Pose2d(FieldConstants.Community.chargingStationCorners[2], Rotation2d.fromDegrees(90))));
    for (int i = 0; i < 4; i++) {
      System.out.println(FieldConstants.Community.chargingStationCorners[i]);
    }
  }
}
