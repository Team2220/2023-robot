// package frc.twilight.swerve.subsystems;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.twilight.swerve.config.PIDconfig;
// import frc.twilight.swerve.vectors.Position;

// public class AdvAuto extends SubsystemBase {
//     private final Swerve kSwerve;

//     private PIDController kPidX = new PIDController(
//             PIDconfig.DT_AUTO_P.getValue(),
//             PIDconfig.DT_AUTO_I.getValue(),
//             PIDconfig.DT_AUTO_D.getValue());
//     private PIDController kPidY = new PIDController(
//             PIDconfig.DT_AUTO_P.getValue(),
//             PIDconfig.DT_AUTO_I.getValue(),
//             PIDconfig.DT_AUTO_D.getValue());
//     private PIDController kPidR = new PIDController(
//             PIDconfig.DT_AUTO_ROT_P.getValue(),
//             PIDconfig.DT_AUTO_ROT_I.getValue(),
//             PIDconfig.DT_AUTO_ROT_D.getValue());

//     private double kXt = 0;
//     private double kYt = 0;
//     private double kRt = 0;

//     private boolean kXRunning = false;
//     private boolean kYRunning = false;
//     private boolean kRRunning = false;

//     private double kXHoldPose = Double.NaN;
//     private double kYHoldPose = Double.NaN;
//     private double kRHoldPose = Double.NaN;

//     private TrapezoidProfile kXProfile;
//     private TrapezoidProfile kYProfile;
//     private TrapezoidProfile kRProfile;

//     private TrapezoidProfile.Constraints kXConstraints = new Constraints(3, 60);
//     private TrapezoidProfile.Constraints kYConstraints = new Constraints(3, 60);
//     private TrapezoidProfile.Constraints kRConstraints = new Constraints(360, 2000);

//     /**
//      * Creates a new Advanced Auto, a way of having more full control over the
//      * swerve base.
//      * 
//      * @param swerve The swerve controlled by the auto
//      */
//     public AdvAuto(Swerve swerve) {
//         kSwerve = swerve;
//     }

//     @Override
//     public void periodic() {
//         double xOut;
//         if (kXRunning) {
//             // Check if the robot has reached its end pos
//                 // If it has, set the running to false, and set

//             kXt += 0.02;
            
//         }
//     }

//     /**
//      * Sets the movement of one of the robot axis.
//      */
//     public CommandBase setPose(Position pose, Axis axis) {
//         return runOnce(
//                 () -> {
//                     switch (axis) {
//                         case x:
//                             setX(pose.getX());
//                             break;
//                         case y:
//                             setY(pose.getY());
//                             break;
//                         case r:
//                             setR(pose.getAngle());
//                             break;
//                     }
//                 });
//     }

//     public CommandBase setPose(Pose2d pose, Axis axis) {
//         return setPose(new Position(pose), axis);
//     }

//     /**
//      * 
//      * @param pose
//      * @param axis
//      * @param greaterThan Ends when pose supplied is greater than current pose.
//      * @return
//      */
//     public CommandBase waitForPose(Position pose, Axis axis, boolean greaterThan) {
//         double end;
//         DoubleSupplier current;
//         switch (axis) {
//             case x:
//                 end = pose.getX();
//                 current = () -> kXProfile.calculate(kXt).position;
//                 break;
//             case y:
//                 end = pose.getY();
//                 current = () -> kYProfile.calculate(kYt).position;
//                 break;
//             case r:
//                 end = pose.getAngle();
//                 current = () -> kRProfile.calculate(kRt).position;
//                 break;
//             default:
//                 return null;
//         }

//         BooleanSupplier condition;

//         if (greaterThan)
//             condition = () -> (current.getAsDouble() < end);
//         else
//             condition = () -> (current.getAsDouble() > end);

//         return Commands.waitUntil(condition);
//     }

//     public CommandBase waitForPose(Pose2d pose, Axis axis, boolean greaterThan) {
//         return waitForPose(new Position(pose), axis, greaterThan);
//     }

//     private void setX(double x) {
//         TrapezoidProfile.State start;
//         if (kXRunning) {
//             start = kXProfile.calculate(kXt);
//         } else {
//             start = new State(kSwerve.getOdo().getX(), kSwerve.getDrive().getStr());
//         }

//         TrapezoidProfile.State end = new State(x, 0);
//         kXProfile = new TrapezoidProfile(kXConstraints, end, start);
//         kXt = 0;
//         kXRunning = true;
//         kXHoldPose = Double.NaN;
//     }

//     private void setY(double y) {
//         TrapezoidProfile.State start;
//         if (kYRunning) {
//             start = kYProfile.calculate(kYt);
//         } else {
//             start = new State(kSwerve.getOdo().getY(), kSwerve.getDrive().getFwd());
//         }

//         TrapezoidProfile.State end = new State(y, 0);
//         kYProfile = new TrapezoidProfile(kYConstraints, end, start);
//         kYt = 0;
//         kYRunning = true;
//         kYHoldPose = Double.NaN;
//     }

//     private void setR(double r) {
//         TrapezoidProfile.State start;
//         if (kRRunning) {
//             start = kRProfile.calculate(kRt);
//         } else {
//             start = new State(kSwerve.getOdo().getAngle(), kSwerve.getDrive().getRcw());
//         }

//         TrapezoidProfile.State end = new State(r, 0);
//         kRProfile = new TrapezoidProfile(kRConstraints, end, start);
//         kRt = 0;
//         kRRunning = true;
//     }

//     /**
//      * Available axis to control.
//      */
//     public static enum Axis {
//         x, y, r
//     }
// }
