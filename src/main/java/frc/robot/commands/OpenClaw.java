package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase{
    
    private final Claw m_ClawSystem;

    public OpenClaw(Claw theClaw) {
        m_ClawSystem = theClaw;
        addRequirements(m_ClawSystem);
    }

    @Override
    public void execute() {
        m_ClawSystem.openClaw();
    }

}