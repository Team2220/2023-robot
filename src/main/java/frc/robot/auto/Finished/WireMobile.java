// Half of robot: 16.291382

package frc.robot.auto.Finished;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.SimpleAutos.JustScoreTheCubeHigh;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.robot.subsystems.Arm;

public class WireMobile extends SequentialCommandGroup {
        public WireMobile(Swerve swerve, Intake intake, Arm arm) {
                addCommands(
                                new JustScoreTheCubeHigh(swerve, arm, intake));

                addCommands(
                                new SetArmState(ArmStates.TRANSIT, arm));
                // // new AutoIntake(-.5, intake).withTimeout(.2),
                // new AutoIntake(.75, intake).withTimeout(.5));

                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(60.5), Units.inchesToMeters(-10),
                                                                Rotation2d.fromDegrees(180))));

                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(-10),
                                                                Rotation2d.fromDegrees(
                                                                                180))));
        }
}
