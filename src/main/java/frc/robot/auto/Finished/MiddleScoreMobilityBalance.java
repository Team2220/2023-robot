package frc.robot.auto.Finished;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.ArmPosition;
import frc.robot.commands.Arm.AutoShoulderState;
import frc.robot.commands.Arm.AutoWristState;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class MiddleScoreMobilityBalance extends SequentialCommandGroup {
  public MiddleScoreMobilityBalance(Swerve swerve, Arm arm, Intake intake) {
    addCommands(
        new InstantCommand(
            () -> swerve.setPose2d(
                new Pose2d(
                    new Translation2d(1.380776, 2.747899),
                    Rotation2d.fromDegrees(180)))),
                    new AutoShoulderState(ArmStates.MID_CUBE_NODE, arm).withTimeout(1),
                    new AutoWristState(ArmStates.MID_CUBE_NODE, arm).withTimeout(1),
                    new AutoIntake(.5, intake).withTimeout(1),   
                    new  GoToCommand(swerve, new Pose2d())
    );
}
}