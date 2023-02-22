package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.twilight.swerve.devices.Gyro;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.tunables.TunableDouble;

public class Balancing extends CommandBase {
  private PIDController pid = new PIDController(0, 0, 0);
  private Swerve swerve;
  private Gyro gyro = new Gyro();

  private static TunableDouble p = new TunableDouble("P", 0, true, "Balancing");
  private static TunableDouble i = new TunableDouble("I", 0, true, "Balancing");
  private static TunableDouble d = new TunableDouble("D", 0, true, "Balancing");

  public Balancing(Swerve swerve) {
    this.swerve = swerve;
    this.addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.setDrive(0, 0, 0);
  }

  @Override
  public void execute() {
    pid.setPID(p.getValue(), i.getValue(), d.getValue());
    double out = gyro.getYRot();
    out = pid.calculate(out, 0);
    swerve.setDrive(0, out, 0);
  }

  @Override
  public void end(boolean interupted) {
    swerve.setDrive(0, 0, 0);
  }
}
