package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ArmSlow extends InstantCommand {
    private final Arm m_ArmSubsystem;

    public ArmSlow(Arm theArmSubsystem) {
        m_ArmSubsystem = theArmSubsystem;
        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.slowArm = true;
    }
}
