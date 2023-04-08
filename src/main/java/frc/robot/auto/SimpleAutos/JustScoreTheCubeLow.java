package frc.robot.auto.SimpleAutos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class JustScoreTheCubeLow extends SequentialCommandGroup {
    public JustScoreTheCubeLow(Swerve swerve, Arm arm, Intake intake) {
        addCommands(
            new InstantCommand(() -> swerve.setOdo(0, 0, 180)),
            new AutoIntake(-0.5, intake).withTimeout(0.4),
            new GoToCommand(swerve, new Position(0, 0.2, 180)),
            new SetArmState(ArmStates.CALIBRATE, arm),
            new InstantCommand(() -> arm.setShoulderFromAbsEncoder()),
            new InstantCommand(() -> arm.setWristFromAbsEncoder()),
            new SetArmState(ArmStates.INTAKE, arm),
            // new GoToCommand(swerve, new Position(0, 0, 180)),
            new AutoIntake(0.5, intake).withTimeout(0.5)
        );
    }
}
