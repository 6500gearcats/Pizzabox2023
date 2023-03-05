package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveSlow extends CommandBase{
    private final DriveSubsystem m_DriveSubsystem;

    public DriveSlow(DriveSubsystem theDriveSubsystem) {
        m_DriveSubsystem = theDriveSubsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.slowTrue();
    }

    @Override
    public void end(boolean done) {
        m_DriveSubsystem.slowFalse();
    }
}
