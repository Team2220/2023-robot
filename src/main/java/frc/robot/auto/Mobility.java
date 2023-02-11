package frc.robot.auto;

import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.util.GeomUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;

public class Mobility extends SequentialCommandGroup {
  public Mobility(Swerve swerve) {

    swerve.setPose2d(
        GeomUtil.translationToPose(FieldConstants.Community.chargingStationCorners[0]));

    addCommands(
        new GoToCommand(
            swerve,
            GeomUtil.translationToPose(FieldConstants.Community.chargingStationCorners[2])));
  }
}
