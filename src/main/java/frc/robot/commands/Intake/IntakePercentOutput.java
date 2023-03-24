package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakePercentOutput extends CommandBase {
  private final Intake m_intake;
  private final DoubleSupplier intakeSpeed;
  private final DoubleSupplier outakeSpeed;

  public IntakePercentOutput(
      DoubleSupplier intakeSpeed, DoubleSupplier outakeSpeed, Intake intake) {
    m_intake = intake;
    addRequirements(intake);
    this.intakeSpeed = intakeSpeed;
    this.outakeSpeed = outakeSpeed;
  }

  @Override
  public void execute() {
    if (intakeSpeed.getAsDouble() > 0.1) {
      m_intake.setPercentOutput(intakeSpeed.getAsDouble());
    } else if (outakeSpeed.getAsDouble() > 0.1) {
      m_intake.setPercentOutput(-outakeSpeed.getAsDouble());
    } else {
      m_intake.setPercentOutput(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setPercentOutput(0);
  }
}
