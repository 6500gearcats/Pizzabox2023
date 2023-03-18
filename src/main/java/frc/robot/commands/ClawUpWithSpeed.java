package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class ClawUpWithSpeed extends CommandBase{
    
    private final Claw m_ClawSystem;
    private double m_ClawSpeed;
    private Supplier<Boolean> m_SlowMode;


    public ClawUpWithSpeed(Claw theClaw, Supplier<Boolean> slowMode) {
        m_ClawSystem = theClaw;
        m_SlowMode = slowMode;
        addRequirements(m_ClawSystem);
    }

    @Override
    public void execute() {
        if(m_SlowMode.get()) {
            m_ClawSpeed = ClawConstants.kClawForwardSpeed * ClawConstants.kSlowClawModifier;
        }
        else {
            m_ClawSpeed = ClawConstants.kClawForwardSpeed;
        }
        m_ClawSystem.clawUpSpeed(m_ClawSpeed);
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