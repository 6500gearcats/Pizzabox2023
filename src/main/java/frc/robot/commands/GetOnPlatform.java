package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroConstants;

public class GetOnPlatform extends CommandBase{

    private final DriveSubsystem m_drive;
    private final Gyro m_gyro;
    public GetOnPlatform(DriveSubsystem drive, Gyro gyro){
        m_drive = drive;
        m_gyro = gyro;
    }

    @Override
    public void execute(){
        m_drive.drive(0.15, 0, 0, true);
    }


    public boolean isFinished(){
        return (Math.abs(m_gyro.getPitch()) <= GyroConstants.kPlatformLevel);
    }

    public void end() {
        m_drive.drive(0,0,0,true);
    }
}
