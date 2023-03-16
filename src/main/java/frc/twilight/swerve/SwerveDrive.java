package frc.twilight.swerve;

import org.littletonrobotics.frc2023.FieldConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.twilight.helpfulThings.Angles;
import frc.twilight.swerve.config.CANidConfig;
import frc.twilight.swerve.config.GeneralConfig;
import frc.twilight.swerve.config.ModuleConfig;
import frc.twilight.swerve.config.PIDconfig;
import frc.twilight.swerve.devices.Gyro;
import frc.twilight.swerve.vectors.DriveVector;
import frc.twilight.swerve.vectors.Position;
import frc.twilight.swerve.vectors.VectorFactory;
import frc.twilight.swerve.vectors.WheelVector;

public class SwerveDrive {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private PIDController gyroPID =
      new PIDController(
          PIDconfig.DT_GYRO_P.getValue(),
          PIDconfig.DT_GYRO_I.getValue(),
          PIDconfig.DT_GYRO_D.getValue());

  private double odoLastCheck = -1;
  private Position odoPosition;

  private Field2d field = new Field2d();

  public SwerveDrive() {
    frontLeft =
        new SwerveModule(
            CANidConfig.DT_FL_DM,
            CANidConfig.DT_FL_SM,
            CANidConfig.DT_FL_SE,
            ModuleConfig.DT_FL_SE_OFFSET);
    frontRight =
        new SwerveModule(
            CANidConfig.DT_FR_DM,
            CANidConfig.DT_FR_SM,
            CANidConfig.DT_FR_SE,
            ModuleConfig.DT_FR_SE_OFFSET);
    backLeft =
        new SwerveModule(
            CANidConfig.DT_BL_DM,
            CANidConfig.DT_BL_SM,
            CANidConfig.DT_BL_SE,
            ModuleConfig.DT_BL_SE_OFFSET);
    backRight =
        new SwerveModule(
            CANidConfig.DT_BR_DM,
            CANidConfig.DT_BR_SM,
            CANidConfig.DT_BR_SE,
            ModuleConfig.DT_BR_SE_OFFSET);

    odoLastCheck = System.currentTimeMillis();

    Gyro.setPosition(GeneralConfig.DT_START_GYRO);

    odoPosition =
        new Position(
            GeneralConfig.DT_START_X, GeneralConfig.DT_START_Y, GeneralConfig.DT_START_GYRO);

    if (GeneralConfig.SWERVE_SHUFFLEBOARD_ENABLED) {
      Shuffleboard.getTab("Swerve")
          .addNumber("FL Enc Angle", () -> frontLeft.getEncoderAngle())
          .withPosition(0, 0);
      Shuffleboard.getTab("Swerve")
          .addNumber("FR Enc Angle", () -> frontRight.getEncoderAngle())
          .withPosition(0, 1);
      Shuffleboard.getTab("Swerve")
          .addNumber("BL Enc Angle", () -> backLeft.getEncoderAngle())
          .withPosition(0, 2);
      Shuffleboard.getTab("Swerve")
          .addNumber("BR Enc Angle", () -> backRight.getEncoderAngle())
          .withPosition(0, 3);

      Shuffleboard.getTab("Swerve")
          .addNumber("FL Angle", () -> frontLeft.get().getAngle())
          .withPosition(1, 0);
      Shuffleboard.getTab("Swerve")
          .addNumber("FR Angle", () -> frontRight.get().getAngle())
          .withPosition(1, 1);
      Shuffleboard.getTab("Swerve")
          .addNumber("BL Angle", () -> backLeft.get().getAngle())
          .withPosition(1, 2);
      Shuffleboard.getTab("Swerve")
          .addNumber("BR Angle", () -> backRight.get().getAngle())
          .withPosition(1, 3);

      Shuffleboard.getTab("Swerve")
          .addNumber("FL Vel", () -> frontLeft.get().getVelocity())
          .withPosition(2, 0);
      Shuffleboard.getTab("Swerve")
          .addNumber("FR Vel", () -> frontRight.get().getVelocity())
          .withPosition(2, 1);
      Shuffleboard.getTab("Swerve")
          .addNumber("BL Vel", () -> backLeft.get().getVelocity())
          .withPosition(2, 2);
      Shuffleboard.getTab("Swerve")
          .addNumber("BR Vel", () -> backRight.get().getVelocity())
          .withPosition(2, 3);

      Shuffleboard.getTab("Swerve").addNumber("X Pos", () -> odoPosition.getX()).withPosition(4, 3);
      Shuffleboard.getTab("Swerve").addNumber("Y Pos", () -> odoPosition.getY()).withPosition(5, 3);

      SmartDashboard.putData("Field", field);

      Shuffleboard.getTab("Swerve").addNumber("Gyro", () -> Gyro.getAngle()).withPosition(6, 3);

      Shuffleboard.getTab("Swerve")
          .addNumber("Gyro Rate", () -> Gyro.getAngleSpeed())
          .withPosition(7, 3);
      Shuffleboard.getTab("Swerve").addBoolean("Gyro is connected", () -> Gyro.isConnected());
    }
  }

  public void setDrive(DriveVector vector, boolean robotCentric) {
    gyroPID.setP(PIDconfig.DT_GYRO_P.getValue());
    gyroPID.setI(PIDconfig.DT_GYRO_I.getValue());
    gyroPID.setD(PIDconfig.DT_GYRO_D.getValue());

    gyroPID.setSetpoint(vector.getRcw());
    vector.setRcw(
        -gyroPID.calculate(Gyro.getAngleSpeed())
            + PIDconfig.DT_GYRO_F.getValue() * vector.getRcw());

    if (!robotCentric)
      vector.zeroDirection(-Gyro.getAngle());

    WheelVector[] wheelVectors = VectorFactory.wheelVectorsFromDriveVector(vector);

    wheelVectors[0] = Angles.optimizeWheel(frontRight.get(), wheelVectors[0]);
    wheelVectors[1] = Angles.optimizeWheel(frontLeft.get(), wheelVectors[1]);
    wheelVectors[2] = Angles.optimizeWheel(backRight.get(), wheelVectors[2]);
    wheelVectors[3] = Angles.optimizeWheel(backLeft.get(), wheelVectors[3]);

    if (Math.abs(wheelVectors[0].getVelocity()) > 0.02
        || Math.abs(wheelVectors[1].getVelocity()) > 0.02
        || Math.abs(wheelVectors[2].getVelocity()) > 0.02
        || Math.abs(wheelVectors[3].getVelocity()) > 0.02) {
      frontRight.set(wheelVectors[0]);
      frontLeft.set(wheelVectors[1]);
      backRight.set(wheelVectors[2]);
      backLeft.set(wheelVectors[3]);
    } else {
      frontRight.stop();
      frontLeft.stop();
      backRight.stop();
      backLeft.stop();
    }
  }

  public void setDrive(DriveVector vector) {
    setDrive(vector, false);
  }

  public DriveVector getDrive() {
    DriveVector out =
        VectorFactory.driveVectorFromWheelVectors(
            frontRight.get(), frontLeft.get(), backRight.get(), backLeft.get());
    out.zeroDirection(-Gyro.getAngle());

    return out;
  }

  public void xMode() {
    frontLeft.set(new WheelVector(0, -45));
    frontRight.set(new WheelVector(0, 45));
    backLeft.set(new WheelVector(0, 45));
    backRight.set(new WheelVector(0, -45));
  }

  public void setOdo(double x, double y, double angle) {
    if (DriverStation.getAlliance() == Alliance.Blue) 
      odoPosition = new Position(x, y, angle);
    else
      odoPosition = new Position(FieldConstants.fieldWidth - x, y, angle);
    Gyro.setPosition(angle);
  }

  public void updateOdo() {
    double time = System.currentTimeMillis();
    double dt = (time - odoLastCheck) / 1000;
    odoLastCheck = time;

    DriveVector drive = getDrive();

    double x = odoPosition.getX() + drive.getFwd() * dt;
    double y = odoPosition.getY() + drive.getStr() * dt;
    double angle = Gyro.getAngle();

    odoPosition = new Position(x, y, angle);

    field.setRobotPose(x, y, new Rotation2d(Units.degreesToRadians(angle)));
  }

  public Position getOdo() {
    return odoPosition;
  }

  public void zeroGyro() {
    Gyro.zeroSensor();
    setOdo(0, 0, 0);
  }
}
