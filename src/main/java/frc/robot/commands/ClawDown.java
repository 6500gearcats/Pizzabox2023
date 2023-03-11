package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
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

    //ends command if claw is past lower limit
    public boolean isFinished() {
        if(m_ClawSystem.getClawAngle() < ClawConstants.kClawLowerLimit) {
            return true;
        }
        return false;
    }

    public void end(boolean done) {
        m_ClawSystem.stopClawTilt();
    }

}