package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class ScoreMid extends CommandBase {
    
    private final Arm m_ArmSystem;
    private final Claw m_ClawSystem;

    public ScoreMid(Arm theArm, Claw theClaw) {
        m_ArmSystem = theArm;
        m_ClawSystem = theClaw;
    }

    @Override
    public void initialize() {
        if(m_ArmSystem.getArmAngle() > ArmConstants.kArmMidAngle && m_ClawSystem.getClawAngle() > ClawConstants.kClawMidAngle) {
            m_ArmSystem.armDown();
            m_ClawSystem.clawDown();
        }
        else if(m_ArmSystem.getArmAngle() > ArmConstants.kArmMidAngle && m_ClawSystem.getClawAngle() < ClawConstants.kClawMidAngle){
            m_ArmSystem.armDown();
            m_ClawSystem.clawUp();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmMidAngle && m_ClawSystem.getClawAngle() > ClawConstants.kClawMidAngle) {
            m_ArmSystem.armUp();
            m_ClawSystem.clawDown();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmMidAngle && m_ClawSystem.getClawAngle() < ClawConstants.kClawMidAngle) {
            m_ArmSystem.armUp();
            m_ClawSystem.clawUp();
        }
    }

    @Override
    public void execute() {
        if(m_ArmSystem.getArmAngle() > ArmConstants.kArmMidAngle - 0.03 && m_ArmSystem.getArmAngle() < ArmConstants.kArmMidAngle + 0.03 ) {
            m_ArmSystem.stopArm();
        }
        if(m_ClawSystem.getClawAngle() > ClawConstants.kClawMidAngle - 0.03 && m_ClawSystem.getClawAngle() < ClawConstants.kClawMidAngle + 0.03) {
            m_ClawSystem.stopClawTilt();
        }
    }

    @Override
    public boolean isFinished() {
        return m_ArmSystem.getArmAngle() > ArmConstants.kArmMidAngle - 0.03
               && m_ArmSystem.getArmAngle() < ArmConstants.kArmMidAngle + 0.03 
               && m_ClawSystem.getClawAngle() > ClawConstants.kClawMidAngle - 0.03 
               && m_ClawSystem.getClawAngle() < ClawConstants.kClawMidAngle + 0.03;
    }
    
    @Override
    public void end(boolean done) {
        m_ArmSystem.stopArm();
        m_ClawSystem.stopClaw();
    }

}