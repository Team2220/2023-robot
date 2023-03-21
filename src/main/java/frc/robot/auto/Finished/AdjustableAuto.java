package frc.robot.auto.Finished;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class AdjustableAuto extends SequentialCommandGroup {
    public final String name;
    public AdjustableAuto(
        Swerve swerve, // Req
        Arm arm, // Req
        Intake intake, // Req
        boolean grabPiece, // Req
        ReferencePoint start, // Req
        ScoreGamepiece scoreGamepiece, // Optional, null if none
        Direction moveDir, // Req
        GetGamepiece getGamepiece, // Optional, null if none
        boolean balance // Req
    ) {
        if (
            swerve == null      ||
            arm == null         ||
            intake == null      ||
            start == null       ||
            moveDir == null
        ) throw new IllegalArgumentException("Null values passed which must not be null! Swerve, Arm, Intake, Starting Ref, and Move Direction are all required!");

        name = "hi";

        /*
         * Suck in the preloaded piece.
         */
        if (grabPiece)
            addCommands(new AutoIntake(-0.5, intake).withTimeout(0.2));

        /*
         * Set the starting point for the robot on the field.
         */
        double xStart = start.xOffset;
        double yStart = Units.inchesToMeters(54.05) + (Constants.ROBOT_Y_LENGTH / 2);

        double lastX = xStart;
        double lastY = yStart;

        addCommands(
            new InstantCommand(() -> swerve.setOdo(xStart, yStart, 180))
        );

        /*
         * Score the game piece
         */
        if (scoreGamepiece != null) {
            if (scoreGamepiece.moveAway) {
                lastX = xStart;
                lastY = yStart + 0.25;
                addCommands(new GoToCommand(swerve, new Position(xStart, yStart + 0.25, 180)));
            }
            
            addCommands(
                new SetArmState(
                    (start.cube ? scoreGamepiece.armstateCube : scoreGamepiece.armstateCone), arm));

            if (scoreGamepiece.moveTowards) {
                lastX = xStart;
                lastY = yStart;
                addCommands(new GoToCommand(swerve, new Position(xStart, yStart, 180)));
            }

            addCommands(
                new AutoIntake(0.5, intake).withTimeout(0.5)
            );
        }

        /*
         * Go to next positon to either grab a piece or balance
         */
        

        /* 
         * Grab second piece.
         */
        if (getGamepiece != null) {
            
        }
    }
    
    public static enum ReferencePoint {
        cubeA(true, Units.inchesToMeters(33 + (18.25 / 2))),
        cubeB(true, Units.inchesToMeters(99 + (18.25 / 2))),
        cubeC(true, Units.inchesToMeters(165 + (18.25 / 2))),
        coneA(false, Units.inchesToMeters(20)),
        coneB(false, Units.inchesToMeters(64)),
        coneC(false, Units.inchesToMeters(86)),
        coneD(false, Units.inchesToMeters(130)),
        coneE(false, Units.inchesToMeters(152)),
        coneF(false, Units.inchesToMeters(196));

        public final boolean cube;
        public final double xOffset;
        private ReferencePoint(boolean cube, double xOffset) {
            this.cube = cube;
            this.xOffset = -xOffset;
        }
    }

    public static enum ScoreGamepiece {
        high(ArmStates.HIGH_CUBE_NODE, ArmStates.HIGH_CONE_NODE, true, true),
        mid(ArmStates.MID_CUBE_NODE, ArmStates.MID_CONE_NODE, true, true),
        low(ArmStates.INTAKE, ArmStates.INTAKE, true, false);

        public final ArmStates armstateCube;
        public final ArmStates armstateCone;
        public final boolean moveAway;
        public final boolean moveTowards;
        private ScoreGamepiece(ArmStates cube, ArmStates cone, boolean moveAway, boolean moveTowards) {
            armstateCube = cube;
            armstateCone = cone;
            this.moveAway = moveAway;
            this.moveTowards = moveTowards;
        }
    }

    public static enum GetGamepiece {
        pieceA(Units.inchesToMeters(36.19), Units.inchesToMeters(278.05)),
        pieceB(Units.inchesToMeters(84.19), Units.inchesToMeters(278.05)),
        pieceC(Units.inchesToMeters(132.19), Units.inchesToMeters(278.05)),
        pieceD(Units.inchesToMeters(180.19), Units.inchesToMeters(278.05));

        public final double x;
        public final double y;
        private GetGamepiece(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static enum Direction {
        TowardsFMS(Units.inchesToMeters(59.39 / 2), true), // Center of area between elements
        AwayFMS(Units.inchesToMeters((216.03 - (59.39 / 2))), false); // Center of area between elements

        public final double xPos;
        public final boolean cableProt;
        private Direction(double x, boolean cable) {
            xPos = x;
            cableProt = cable;
        }
    }
}
