package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawDown extends CommandBase{
    
    private final Claw m_ClawSystem;

    public ClawDown(Claw theClaw) {
        m_ClawSystem = theClaw;
    }

    @Override
    public void execute() {
        m_ClawSystem.clawDown();
    }

}