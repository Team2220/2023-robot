package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.DriveVector;
import frc.twilight.tunables.TunableDouble;

public class ObjectTracker extends CommandBase {
  private PIDController pid = new PIDController(0, 0, 0);

  private Swerve swerve;

  private static TunableDouble p = new TunableDouble("P", 12, true, "limelight");
  private static TunableDouble i = new TunableDouble("I", 0, true, "limelight");
  private static TunableDouble d = new TunableDouble("D", 0, true, "limelight");
  private final DoubleSupplier fwd;
  private final DoubleSupplier str;

  public ObjectTracker(Swerve swerve, DoubleSupplier fwd, DoubleSupplier str) {
    this.swerve = swerve;
    this.addRequirements(swerve);
    this.fwd = fwd;
    this.str = str;

    pid.setTolerance(0.5);
    pid.setSetpoint(0);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    pid.setPID(p.getValue(), i.getValue(), d.getValue());

    double out = 0;
    boolean foundCone = false;

    var results = LimelightHelpers.getLatestResults("limelight-right");
    for (LimelightTarget_Detector target : results.targetingResults.targets_Detector) {
      if (target.className.equals("cone")) {
        out = target.tx;
        foundCone = true;
      }
    }

    out = pid.calculate(out);

    if (foundCone) {
      System.out.println("out: " + out);
      // out = MathUtil.clamp(out, -1, 1);
      swerve.setDrive(new DriveVector(fwd.getAsDouble(), str.getAsDouble(), -out), true);
    } else {
      swerve.setDrive(new DriveVector(fwd.getAsDouble(), str.getAsDouble(), 0));
    }

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
