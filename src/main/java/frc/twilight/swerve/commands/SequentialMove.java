package frc.twilight.swerve.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.twilight.swerve.subsystems.Swerve;
import frc.twilight.swerve.vectors.Position;

public class SequentialMove extends SequentialCommandGroup {
    public SequentialMove(Swerve swerve, double radius, double endMovTol, double endRotTol, boolean stopAtEnd, Position... positions) {
        // if (positions.length < 1)
        //     return;

        GoToCommand[] commands = new GoToCommand[positions.length];
        
        for (int i = 0; i < positions.length; i++)
            commands[i] = 
                new GoToCommand(swerve, positions[i])
                    .stopAtEnd(false)
                    .setTolerence(radius, -1); 

        commands[commands.length - 1].stopAtEnd(stopAtEnd).setTolerence(endMovTol, endRotTol);

        addCommands(commands);
    }

    public SequentialMove(Swerve swerve, Position... positions) {
        this(swerve, 0.75, 0.05, 2.5, true, positions);
    }
}
