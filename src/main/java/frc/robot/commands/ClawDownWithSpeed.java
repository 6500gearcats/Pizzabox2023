package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class ClawDownWithSpeed extends CommandBase{
    
    private final Claw m_ClawSystem;
    private double m_ClawSpeed;

    public ClawDownWithSpeed(Claw theClaw, double speed) {
        m_ClawSpeed = speed;
        m_ClawSystem = theClaw;
        addRequirements(m_ClawSystem);
    }

    @Override
    public void execute() {
        m_ClawSystem.clawDownSpeed(m_ClawSpeed);
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