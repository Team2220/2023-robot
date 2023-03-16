package frc.robot.auto.Finished;
//working

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;

public class MobilityL extends SequentialCommandGroup {
    public MobilityL(Swerve swerve, Arm arm, Intake intake) {
        addCommands(
                new InstantCommand(
                        () -> swerve.setPose2d(
                                new Pose2d(
                                        new Translation2d(Units.inchesToMeters(54.361250 + 16.291382),
                                                Units.inchesToMeters(174.185000)),
/*set stating position*/                            Rotation2d.fromDegrees(180)))),


                new AutoIntake(-.5, intake).withTimeout(.2),
                new SetArmState(ArmStates.MID_CUBE_NODE, arm),
                new AutoIntake(.5, intake).withTimeout(1),
                new SetArmState(ArmStates.TRANSIT, arm),
                new GoToCommand(swerve, new Pose2d((3.311176 + 0.4138011028), Units.inchesToMeters(174.185000), 
                Rotation2d.fromDegrees(0)))

        );
    }
}