package frc.robot.auto.Finished;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balancing;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class MidScore1BalBlueAutoL extends SequentialCommandGroup {
  public MidScore1BalBlueAutoL(Swerve swerve, Arm arm, Intake intake) {
    addCommands(
      new InstantCommand(() -> swerve.setOdo(Units.inchesToMeters(54.361250 + 16.291382), Units.inchesToMeters(174.185000), 180)), // set starting position
 // score starting game piece
        new SetArmState(ArmStates.MID_CUBE_NODE, arm),
        new AutoIntake(.5, intake).withTimeout(1),
        new SetArmState(ArmStates.INTAKE, arm),
        new GoToCommand(swerve, new Pose2d(7.070376, 4.576699, Rotation2d.fromDegrees(0))).alongWith(new AutoIntake(-.5, intake).withTimeout(1)),
        new GoToCommand(swerve, new Pose2d(Units.inchesToMeters(54.361250 + 16.291382), Units.inchesToMeters(174.185000), 180)),
        new SetArmState(ArmStates.MID_CUBE_NODE, arm),
        new AutoIntake(.5, intake).withTimeout(1),
        new GoToCommand(
                        swerve,
                        new Pose2d(1.9453771028, 2.743581,
                                Rotation2d.fromDegrees(0))),
                new GoToCommand(
                        swerve,
                        new Pose2d(3.279775, 2.743581,
                                Rotation2d.fromDegrees(180))),
                
        // balance

         
        new Balancing(swerve));
  }

  
}