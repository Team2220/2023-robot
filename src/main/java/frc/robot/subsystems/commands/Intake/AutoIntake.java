package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutoIntake extends CommandBase {
    private final Intake m_intake;
    private final double speed;

    public AutoIntake(double speed, Intake m_intake) {
        this.m_intake = m_intake;
        this.addRequirements(m_intake);
        this.speed = speed;
    }

    @Override
    public void execute() {
        m_intake.setPercentOutput(speed);
    }

    @Override
    public void end(boolean interrupted) {
      m_intake.setPercentOutput(0);
    }
}
