package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase{
    
    private final Claw m_ClawSystem;

    public CloseClaw(Claw theClaw) {
        m_ClawSystem = theClaw;
        addRequirements(m_ClawSystem);
    }

    @Override
    public void execute() {
        m_ClawSystem.closeClaw();
    }

}