package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class ClawUp extends CommandBase{
    
    private final Claw m_ClawSystem;

    public ClawUp(Claw theClaw) {
        m_ClawSystem = theClaw;
    }

    @Override
    public void execute() {
        if(m_ClawSystem.getClawAngle() < ClawConstants.kClawLowerLimit) {
            m_ClawSystem.clawUp();
        }
    }

    public void end(boolean done) {
        m_ClawSystem.stopClawTilt();
    }
}
