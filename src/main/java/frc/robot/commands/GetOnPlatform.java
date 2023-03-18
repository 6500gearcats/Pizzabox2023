package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroConstants;

public class GetOnPlatform extends CommandBase{

    private final DriveSubsystem m_drive;
    public GetOnPlatform(DriveSubsystem drive){
        m_drive = drive;
        System.out.println("Got on the platform.");
    }

    @Override
    public void execute(){
        double angle = m_drive.getPitch();
        if(Math.abs(angle) > 1){
            // Drive in the correction direction at kPlatformSpeed m/s
            m_drive.drive(Constants.PlatformConstants.kPlatformSpeed * -1 * ( angle / Math.abs(angle) ), 0, 0, false);
            //m_drive.drive(-0.1*(Math.log(Math.abs(angle))*angle/Math.abs(angle)), 0, 0, true);
        }
    }

   // public boolean isFinished(){
     //   return (Math.abs(m_gyro.getPitch()) <= GyroConstants.kPlatformLevel);
    //}

    public void end() {
        m_drive.drive(0,0,0,true);
    }
}
