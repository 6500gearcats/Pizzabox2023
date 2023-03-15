package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyro;

public class ClimbPlatform extends SequentialCommandGroup{
    public ClimbPlatform(DriveSubsystem drive, Gyro gyro){
        super(
            new FindPlatform(drive, gyro),
            new GetOnPlatform(drive, gyro)
        );
    }
}
