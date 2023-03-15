package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class ClawNormal extends InstantCommand {
    private final Claw m_ClawSubsystem;

    public ClawNormal(Claw theClawSubsystem) {
        m_ClawSubsystem = theClawSubsystem;
        addRequirements(m_ClawSubsystem);
    }

    @Override
    public void initialize() {
        m_ClawSubsystem.slowClaw = false;
    }
}
