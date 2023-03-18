package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTurbo extends InstantCommand {
    private final DriveSubsystem m_DriveSubsystem;

    public DriveTurbo(DriveSubsystem theDriveSubsystem) {
        m_DriveSubsystem = theDriveSubsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.m_turboEnable = true;
    }
}
