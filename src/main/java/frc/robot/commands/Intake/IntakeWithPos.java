package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class IntakeWithPos extends CommandBase {
    public static final double wristAngleIntake = 41;
    public static final double shoulderAngleIntake = 152;

    private double wristAngleOrigin;
    private double shoulderAngleOrigin;

    private double intakeSpeed;

    private Arm arm;
    private Intake intake;

    public IntakeWithPos(double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
    }

    @Override
    public void initialize() {
        wristAngleOrigin = arm.getMotorWristPosition();
        shoulderAngleOrigin = arm.getMotorShoulderPosition();

        new ArmPosition(wristAngleIntake, shoulderAngleIntake, arm).schedule();
        new AutoIntake(intakeSpeed, intake).schedule();
    }

    @Override
    public void end(boolean interupted) {
        new ArmPosition(wristAngleOrigin, shoulderAngleOrigin, arm).schedule();
        new AutoIntake(0, intake).schedule();
    }
}
