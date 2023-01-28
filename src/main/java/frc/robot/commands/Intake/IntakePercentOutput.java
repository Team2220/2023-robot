package frc.robot.commands.Intake;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class IntakePercentOutput extends CommandBase {
    private final Intake m_intake;
    private final double speed;

    public IntakePercentOutput(double speed, Intake intake) {
        m_intake = intake;
        addRequirements(intake);
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
