package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;


public class ClimbPlatform extends SequentialCommandGroup{
    public ClimbPlatform(DriveSubsystem drive){
        addCommands(
            new FindPlatform(drive),
            new RunCommand(() -> drive.drive(.8,0,0,false)).withTimeout(0.5),
            new GetOnPlatform(drive)
            //.andThen(new WaitCommand(0.2)).repeatedly()
        );
    }
}
