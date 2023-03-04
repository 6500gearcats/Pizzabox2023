package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class ScoreHighPosition extends CommandBase {
    
    private final Arm m_ArmSystem;
    private final Claw m_ClawSystem;

    public ScoreHighPosition(Arm theArm, Claw theClaw) {
        m_ArmSystem = theArm;
        m_ClawSystem = theClaw;
    }

    @Override
    public void execute() {
        if(m_ArmSystem.getArmAngle() > ArmConstants.kArmHighAngleMax) {
            m_ArmSystem.armDown();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmHighAngleMin) {
            m_ArmSystem.armDown();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmHighAngleMin) {
            m_ArmSystem.armDown();
        }
        
        if(m_ClawSystem.getClawAngle() > ClawConstants.kClawHighAngleMax) {
            m_ClawSystem.clawUp();
        }
        else if(m_ClawSystem.getClawAngle() < ClawConstants.kClawHighAngleMin) {
            m_ClawSystem.clawUp();
        }
        else if(m_ClawSystem.getClawAngle() > ClawConstants.kClawHighAngleMax && m_ClawSystem.getClawAngle() < ClawConstants.kClawHighAngleMin) {
            m_ClawSystem.stopClaw();
        }
    }

    /*@Override
    public boolean isFinished() {
        if(m_ArmSystem.getArmAngle() == ArmConstants.kArmHighAngle && m_ClawSystem.getClawAngle() == ClawConstants.kClawHighAngle) {
            return true;
        }
        else {
            return false;
        }
    }*/
    
    @Override
    public void end(boolean done) {
        m_ArmSystem.stopArm();
        m_ClawSystem.stopClaw();
    }

}