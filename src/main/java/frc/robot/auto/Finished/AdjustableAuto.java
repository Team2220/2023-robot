package frc.robot.auto.Finished;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SetArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmStates;
import frc.twilight.swerve.commands.GoToCommand;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class AdjustableAuto extends SequentialCommandGroup {
    public final String name;
    public AdjustableAuto(
        Swerve swerve,
        Arm arm, 
        Intake intake,
        ReferencePoint start,
        ScoreGamepiece scoreGamepiece
    ) {
        name = "hi";

        /*
         * Set the starting point for the robot on the field.
         */
        double xStart = start.xOffset;
        double yStart = Units.inchesToMeters(54.05);

        addCommands(
            new InstantCommand(() -> swerve.setOdo(xStart, yStart, 180))
        );

        /*
         * Score the game piece
         */
        if (scoreGamepiece != null) {
            if (scoreGamepiece.moveAway)
                addCommands(new GoToCommand(swerve, new Position(xStart, yStart + 0.25, 180)));
            
            addCommands(
                new SetArmState(
                    (start.cube ? scoreGamepiece.armstateCube : scoreGamepiece.armstateCone), arm));

            if (scoreGamepiece.moveTowards)
                addCommands(new GoToCommand(swerve, new Position(xStart, yStart, 180)));
        }

        /* 
         * 
         */
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
        
    }
}
