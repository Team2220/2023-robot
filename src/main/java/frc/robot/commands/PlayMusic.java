package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;

public class PlayMusic extends CommandBase{
    private final Swerve swerve;
    private final Arm arm; 
    private final Intake intake;

    public PlayMusic(Swerve swerve, Arm arm, Intake intake){
        this.swerve = swerve;
        this.arm = arm;
        this.intake = intake;
        



    }


    
}


