package frc.robot.auto.Finished;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class CenterBalMobility extends SequentialCommandGroup {
    public CenterBalMobility(Swerve swerve, Arm arm, Intake intake) {
        addCommands(
            new JustScoreTheCubeHigh(swerve, arm, intake),
            new GoToCommand(swerve, new Position(0, Units.inchesToMeters(200), 180))
                .alongWith(new SetArmState(ArmStates.TRANSIT, arm).withTimeout(2)),
            new GoToCommand(swerve, new Position(0, Units.inchesToMeters(96.75) + 0.4, 180)),
            new Balancing(swerve)
        );
    }
}
