package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.twilight.swerve.devices.Gyro;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.DriveVector;
import frc.twilight.tunables.TunableDouble;

public class Balancing extends CommandBase {
  private PIDController pid = new PIDController(0, 0, 0);
  private PIDController pidClose = new PIDController(0, 0, 0);
  private Swerve swerve;

  private static TunableDouble p = new TunableDouble("P", 0.02, true, "Balancing");
  private static TunableDouble i = new TunableDouble("I", 0.005, true, "Balancing");
  private static TunableDouble d = new TunableDouble("D", 0.000003, true, "Balancing");

  private static TunableDouble p2 = new TunableDouble("PC", 0, true, "Balancing");
  private static TunableDouble i2 = new TunableDouble("IC", 0, true, "Balancing");
  private static TunableDouble d2 = new TunableDouble("DC", 0, true, "Balancing");

  private static boolean shuffled = false;

  public Balancing(Swerve swerve) {
    this.swerve = swerve;
    this.addRequirements(swerve);

    if (!shuffled) {
      Shuffleboard.getTab("Balancing").addNumber("Y Rot Angle", () -> Gyro.getYRot());
      shuffled = true;
    }

    pid.setTolerance(0.5);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pid.setPID(p.getValue(), i.getValue(), d.getValue());
    pidClose.setPID(p2.getValue(), i2.getValue(), d2.getValue());

    double out = Gyro.getYRot();
    
    if (Math.abs(out - 5.7) < 1) {
      swerve.xMode();
      return;
    } else if (Math.abs(out - 5.7) < 4)
      out = pidClose.calculate(out, 5.7);
    else
      out = pid.calculate(out, 5.7);

    // out = MathUtil.clamp(out, -1, 1);
    swerve.setDrive(new DriveVector(-out, 0, 0), true);
  }

  @Override
  public void end(boolean interupted) {
    swerve.setDrive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
