package frc.robot.auto.Finished;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class ScoreAndGetC extends SequentialCommandGroup {
    public ScoreAndGetC(Swerve swerve, Arm arm, Intake intake) {
        addCommands(
            new JustScoreTheCubeHigh(swerve, arm, intake),
            new GoToCommand(swerve, new Position(0, .2, 180)),
            new SetArmState(ArmStates.TRANSIT, arm),
            new GoToCommand(swerve, new Position(0.3, 0.5, 180)),
            new GoToCommand(swerve, new Position(
                0.3,
                Units.inchesToMeters(200.0) - (Constants.ROBOT_Y_LENGTH / 2),
                0
            )),
            new SetArmState(ArmStates.INTAKE, arm),
            new GoToCommand(swerve, new Position(
                0, 
                Units.inchesToMeters(224) - (Constants.ROBOT_Y_LENGTH / 2), 
                0
            )).raceWith(new AutoIntake(-0.5, intake))
        );
    }
}
