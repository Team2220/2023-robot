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
  private Swerve swerve;

  private static TunableDouble p = new TunableDouble("P", 0, true, "Balancing");
  private static TunableDouble i = new TunableDouble("I", 0, true, "Balancing");
  private static TunableDouble d = new TunableDouble("D", 0, true, "Balancing");

  private static boolean shuffled = false;

  public Balancing(Swerve swerve) {
    this.swerve = swerve;
    this.addRequirements(swerve);

    if (!shuffled) {
      Shuffleboard.getTab("Balancing").addNumber("Y Rot Angle", () -> Gyro.getYRot());
      shuffled = true;
    }
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pid.setPID(p.getValue(), i.getValue(), d.getValue());
    double out = Gyro.getYRot();
    out = pid.calculate(out, 5);
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
