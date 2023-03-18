package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroConstants;

public class FindPlatform extends CommandBase{

    private final DriveSubsystem m_drive;
    public FindPlatform(DriveSubsystem drive){
        m_drive = drive;
    }

    @Override
    public void execute(){
        m_drive.drive(0.85, 0, 0, true);
    }


    public boolean isFinished(){
        //need to test and adjust roll boolean
        return (Math.abs(m_drive.getPitch()) > 15 && Math.abs(m_drive.getRoll()) > 15);
    }
}
