package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GyroConstants;

public class GetOnPlatform extends CommandBase{

    private final DriveSubsystem m_drive;
    private boolean m_lastAngle;
    private double angleSignPos;
    public GetOnPlatform(DriveSubsystem drive){
        m_drive = drive;
        System.out.println("Got on the platform.");
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("GoP Ended", false);
        m_lastAngle = false;
    }


    @Override
    public void execute(){
        double angle = m_drive.getPitch();
        angleSignPos = Math.signum(angle);
        if(Math.abs(angle) > GyroConstants.kPlatformLevel){
            // Drive in the correction direction at kPlatformSpeed m/s
            m_drive.drive(Constants.PlatformConstants.kPlatformSpeed * -1 * ( angle / Math.abs(angle) ), 0, 0, false);
            //m_drive.drive(-0.1*(Math.log(Math.abs(angle))*angle/Math.abs(angle)), 0, 0, true);
        } else {
            m_drive.drive(0, 0, 0, false);
        }

        if(angleSignPos > 0)
        {
            m_lastAngle = true;
        }
    }

    @Override
    public boolean isFinished(){
        //return (Math.abs(m_drive.getPitch()) <= GyroConstants.kPlatformLevel);
        return m_lastAngle != (m_drive.getPitch() > 0);
    }

    @Override
    public void end(boolean done) {
        //m_drive.drive(0,0,0,false);
        SmartDashboard.putBoolean("GoP Ended", true);
        m_drive.setX();
    }
}