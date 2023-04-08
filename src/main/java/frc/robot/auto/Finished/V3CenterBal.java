package frc.robot.auto.Finished;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.SimpleAutos.JustScoreTheCubeHigh;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;

public class V3CenterBal extends SequentialCommandGroup {
        public V3CenterBal(Swerve swerve, Arm arm, Intake intake) {
                addCommands(
                                new InstantCommand(
                                                () -> swerve.setPose2d(
                                                                new Pose2d(
                                                                                new Translation2d(1.8453771028,
                                                                                                2.747899),
                                                                                Rotation2d.fromDegrees(180)))), // set
                                                                                                                // starting
                                                                                                                // position
                                // score starting game piece
                                new JustScoreTheCubeHigh(swerve, arm, intake),
                                // score game piece 1
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(2.952708,
                                                                2.747899 + 0.377830 + 1.216025 + 0.379072
                                                                                + 0.4138011028,
                                                                Rotation2d.fromDegrees(0)))
                                                .alongWith(new SetArmState(ArmStates.TRANSIT, arm)),
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(7.057840, 2.134508,
                                                                Rotation2d.fromDegrees(0)))
                                                .alongWith(new SetArmState(ArmStates.INTAKE, arm))
                                                .raceWith(new AutoIntake(-.5, intake).withTimeout(2.5)),
                                new SetArmState(ArmStates.TRANSIT, arm)
                                                .raceWith(new AutoIntake(-.5, intake).withTimeout(.2))
                                                .alongWith(new GoToCommand(
                                                                swerve,
                                                                new Pose2d(2.952708,
                                                                                2.747899 + 0.377830 + 2.747899
                                                                                                + 0.377830,
                                                                                Rotation2d.fromDegrees(0)))),
                                // balance
                                new Balancing(swerve),
                                // if wanting to shoot gamepiece at end of auto
                                new JustScoreTheCubeHigh(swerve, arm, intake));
        }
}