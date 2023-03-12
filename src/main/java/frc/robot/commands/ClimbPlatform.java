package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyro;

public class ClimbPlatform extends SequentialCommandGroup{
    public ClimbPlatform(DriveSubsystem Drive, Gyro Gyro){
        addCommands(
            new FindPlatform(Drive, Gyro),
            new GetOnPlatform(Drive, Gyro)
        );
    }
}
