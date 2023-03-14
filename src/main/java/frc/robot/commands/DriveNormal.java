package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveNormal extends InstantCommand {
    private final DriveSubsystem m_DriveSubsystem;

    public DriveNormal(DriveSubsystem theDriveSubsystem) {
        m_DriveSubsystem = theDriveSubsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.slowEnable = false;
    }
}
