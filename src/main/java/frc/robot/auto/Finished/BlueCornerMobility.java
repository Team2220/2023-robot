// Half of robot: 16.291382

package frc.robot.auto.Finished;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Intake;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;

public class BlueCornerMobility extends SequentialCommandGroup {
        public BlueCornerMobility(Swerve swerve, Intake intake) {
                addCommands(
                                new InstantCommand(
                                                () -> swerve.setPose2d(
                                                                new Pose2d(
                                                                                new Translation2d(0, 0),
                                                                                Rotation2d.fromDegrees(180)))));

                addCommands(
                                new AutoIntake(-.5, intake).withTimeout(2));

                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(60.5), Units.inchesToMeters(-10),
                                                                Rotation2d.fromDegrees(
                                                                                180))));
                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(165), Units.inchesToMeters(-10),
                                                                Rotation2d.fromDegrees(
                                                                                180))));

                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(165), Units.inchesToMeters(78),
                                                                Rotation2d.fromDegrees(
                                                                                180))));

                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(127), Units.inchesToMeters(78),
                                                                Rotation2d.fromDegrees(
                                                                                180))));

                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(75), Units.inchesToMeters(78),
                                                                Rotation2d.fromDegrees(
                                                                                0))));

        }
}
