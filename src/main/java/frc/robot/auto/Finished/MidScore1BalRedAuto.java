// unfinished

package frc.robot.auto.Finished;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.AutoShoulderState;
import frc.robot.commands.Arm.AutoWristState;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.commands.Intake.IntakePercentOutput;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Intake;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class MidScore1BalRedAuto extends SequentialCommandGroup {
    public MidScore1BalRedAuto(Swerve swerve, Arm arm, Intake intake) {
        addCommands(
                new InstantCommand(
                        () -> swerve.setPose2d(
                                new Pose2d(
                                        new Translation2d(1.8453771028, 1.071499),
                                        Rotation2d.fromDegrees(180))))); // set starting position
        addCommands( // score starting game piece
                new AutoShoulderState(ArmStates.MID_CUBE_NODE, arm).withTimeout(1),
                new AutoWristState(ArmStates.MID_CUBE_NODE, arm).withTimeout(1),
                new AutoIntake(.5, intake).withTimeout(1),

                // score game piece 1
                new SetArmState(ArmStates.INTAKE, arm),
                new GoToCommand(
                        swerve,
                        new Pose2d(7.070376, 0.919099,
                                Rotation2d.fromDegrees(0))),
                new AutoIntake(-.5, intake).withTimeout(1),

                new GoToCommand(
                        swerve,
                        new Pose2d(1.8453771028, 1.071499,
                                Rotation2d.fromDegrees(180))),
                new SetArmState(ArmStates.HIGH_CUBE_NODE, arm),
                new AutoIntake(.5, intake).withTimeout(1));
                new SetArmState(ArmStates.TRANSIT, arm);
        addCommands( // balance
                new GoToCommand(
                        swerve,
                        new Pose2d(1.9453771028, 2.743581,
                                Rotation2d.fromDegrees(0))),
                new GoToCommand(
                        swerve,
                        new Pose2d(3.279775, 2.743581,
                                Rotation2d.fromDegrees(180))),
                new Balancing(swerve));
    }
}
