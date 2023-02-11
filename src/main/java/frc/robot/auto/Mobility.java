package frc.robot.auto;

import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.util.GeomUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class Mobility extends SequentialCommandGroup {
    public Mobility(Swerve swerve) {
      addCommands(new InstantCommand(() -> swerve.setPose2d(GeomUtil.translationToPose(FieldConstants.Community.chargingStationCorners[0]))));
      addCommands(new GoToCommand(swerve, GeomUtil.translationToPose(FieldConstants.Community.chargingStationCorners[2])));
      for (int i = 0; i < 4; i++) {
        System.out.println(FieldConstants.Community.chargingStationCorners[i]);
      }
    }
  }