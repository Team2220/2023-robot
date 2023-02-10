package frc.robot.commands;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.twilight.swerve.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;

public class PlayMusic extends CommandBase {
  private final Swerve swerve;
  private final Arm arm;
  private final Intake intake;
  private final Orchestra orchestra;

  public PlayMusic(Swerve swerve, Arm arm, Intake intake) {
    this.swerve = swerve;
    this.arm = arm;
    this.intake = intake;
    this.orchestra = new Orchestra(arm.geTalonFXs());
    addRequirements(swerve);
    addRequirements(intake);
    addRequirements(arm);

    orchestra.loadMusic("song10.chrp");
  }

  @Override
  public void initialize() {
    orchestra.play();
  }

  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }
}
