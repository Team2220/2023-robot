// Half of robot: 16.291382 or 0.4138011028

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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;

public class NonWireMobile extends SequentialCommandGroup {
        public NonWireMobile(Swerve swerve, Intake intake, Arm arm) {
                addCommands(new JustScoreTheCubeHigh(swerve, arm, intake),
                        new SetArmState(ArmStates.TRANSIT, arm));

                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(60.5), Units.inchesToMeters(10),
                                                                Rotation2d.fromDegrees(
                                                                                180))));
                addCommands(
                                new GoToCommand(
                                                swerve,
                                                new Pose2d(Units.inchesToMeters(165), Units.inchesToMeters(10),
                                                                Rotation2d.fromDegrees(
                                                                                180))));
        }
}
