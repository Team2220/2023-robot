package frc.robot.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.ArmPosition;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.twilight.swerve.subsystems.Swerve;

public class TestPath extends SequentialCommandGroup {
  public TestPath(Swerve swerve, Arm arm, Intake intake) {
    // // Set up starting position
    // swerve.setOdo(
    //   Units.inchesToMeters(54.05) + (Constants.ROBOT_X_LENGTH / 2), 
    //   Units.inchesToMeters(42.345), 
    //   180
    // );

    // // Make robot drop off starting cube
    // // Start with shoulder
    // addCommands(new ArmPosition(140, 70, arm));
    // // Then move wrist
    // addCommands(new ArmPosition(-30, 70, arm));
    // // Shoot intake for a half second
    // addCommands(new AutoIntake(-0.5, intake).withTimeout(0.5));

    addCommands(new Balancing(swerve));
  }
}
