package frc.robot.auto.inProgress;

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

public class MidCubeMidCone extends SequentialCommandGroup {
    public MidCubeMidCone (Swerve swerve, Arm arm, Intake intake) {
        addCommands( // This is my(addies)(and now jithins) attempt at an auto lol sorry tim and ryan hopfully you dont disintegrate at the sight of this
            new InstantCommand(() -> swerve.setOdo(0, 0, 180)),
            new AutoIntake(-0.5, intake).withTimeout(0.2),
            new GoToCommand(swerve, new Position(0, 0.2, 180)),
            new SetArmState(ArmStates.MID_CUBE_NODE, arm),
            new GoToCommand(swerve, new Position(0, 0, 180)),
            new AutoIntake(0.5, intake).withTimeout(0.5),
            new GoToCommand(swerve, new Position(0, 0, 0))
                .alongWith(new SetArmState(ArmStates.TRANSIT, arm)),
            // i wanna try to intke a cone if thats ok? maybe?
            new GoToCommand(swerve, new Position(7.070376, 4.576699, 0))
                .alongWith(new SetArmState(ArmStates.INTAKE, arm))
                .raceWith(new AutoIntake(-0.6, intake)),
            new GoToCommand(swerve, new Position(1.8453771028, 4.976749, 180))
                .alongWith(new SetArmState(ArmStates.TRANSIT, arm)),
            new SetArmState(ArmStates.MID_CONE_NODE, arm)
                .alongWith(new AutoIntake(-.5, intake).withTimeout(.2)),
            new AutoIntake(0.5, intake)
        );
    }
}
 