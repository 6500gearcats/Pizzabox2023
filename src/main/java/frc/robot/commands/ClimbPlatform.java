package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;


public class ClimbPlatform extends SequentialCommandGroup{
    public ClimbPlatform(DriveSubsystem Drive){
        addCommands(
            new FindPlatform(Drive),
            new GetOnPlatform(Drive)
        );
    }
}
