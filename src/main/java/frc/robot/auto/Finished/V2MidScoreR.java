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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Intake;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;

public class V2MidScoreR extends SequentialCommandGroup {
    public V2MidScoreR(Swerve swerve, Arm arm, Intake intake) {
        addCommands(
                new JustScoreTheCubeHigh(swerve, arm, intake),

                new InstantCommand(
                        () -> swerve.setPose2d(
                                new Pose2d(
                                        new Translation2d(1.8453771028,
                                                1.071499),
                                        Rotation2d.fromDegrees(180)))), // set
                                                                        // starting
                                                                        // position
                // // score starting game piece
                // new AutoIntake(-.5, intake).withTimeout(.2),
                // new AutoShoulderState(ArmStates.MID_CUBE_NODE, arm).withTimeout(1),
                // new AutoWristState(ArmStates.MID_CUBE_NODE, arm).withTimeout(1),
                // new AutoIntake(.75, intake).withTimeout(1),

                // score game piece 1
                new SetArmState(ArmStates.TRANSIT, arm),
                new GoToCommand(
                        swerve,
                        new Pose2d(4.2238011028, 0.4498141028,
                                Rotation2d.fromDegrees(0))),
                new SetArmState(ArmStates.INTAKE, arm),                
                new GoToCommand(
                        swerve,
                        new Pose2d(7.070376, 0.919099,
                                Rotation2d.fromDegrees(0)))
                        .alongWith(new AutoIntake(-.5, intake).withTimeout(2.5)),
                new SetArmState(ArmStates.TRANSIT, arm)
                .alongWith(new AutoIntake(-.5, intake).withTimeout(.2)),    
                new GoToCommand(
                    swerve,
                    new Pose2d(4.2238011028, 0.4498141028,
                            Rotation2d.fromDegrees(180))),    
                new GoToCommand(
                        swerve,
                        new Pose2d(1.8453771028, 1.071499,
                                Rotation2d.fromDegrees(180))),
                new AutoShoulderState(ArmStates.HIGH_CUBE_NODE, arm).withTimeout(.5),
                new AutoWristState(ArmStates.HIGH_CUBE_NODE, arm).withTimeout(.5),
                new AutoIntake(.5, intake).withTimeout(1),
                new AutoWristState(ArmStates.TRANSIT, arm).withTimeout(1),
                new AutoShoulderState(ArmStates.TRANSIT, arm).withTimeout(1),
                // balance
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
