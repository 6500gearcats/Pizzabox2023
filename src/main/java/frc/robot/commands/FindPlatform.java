package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroConstants;

public class FindPlatform extends CommandBase{

    private final DriveSubsystem m_drive;
    private boolean forwards;
    public FindPlatform(DriveSubsystem drive, boolean forward){
        m_drive = drive;
        forwards = forward;
    }

    @Override
    public void execute(){
        if(forwards) {
            m_drive.drive(0.85, 0, 0, false);
        }
        else {
            m_drive.drive(-.85, 0, 0, false);
        }
    }


    public boolean isFinished(){
        return (Math.abs(m_drive.getPitch()) > 15);
    }
}
