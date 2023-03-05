package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class ScoreLowPosition extends CommandBase {
    
    private final Arm m_ArmSystem;
    private final Claw m_ClawSystem;

    public ScoreLowPosition(Arm theArm, Claw theClaw) {
        m_ArmSystem = theArm;
        m_ClawSystem = theClaw;
    }

    @Override
    public void execute() {
        if(m_ArmSystem.getArmAngle() > ArmConstants.kArmLowAngle && m_ClawSystem.getClawAngle() > ClawConstants.kClawLowAngle) {
            m_ArmSystem.armDown();
            m_ClawSystem.clawDown();
        }
        else if(m_ArmSystem.getArmAngle() > ArmConstants.kArmLowAngle && m_ClawSystem.getClawAngle() < ClawConstants.kClawLowAngle){
            m_ArmSystem.armDown();
            m_ClawSystem.clawUp();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmLowAngle && m_ClawSystem.getClawAngle() > ClawConstants.kClawLowAngle) {
            m_ArmSystem.armUp();
            m_ClawSystem.clawDown();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmLowAngle && m_ClawSystem.getClawAngle() < ClawConstants.kClawLowAngle) {
            m_ArmSystem.armUp();
            m_ClawSystem.clawUp();
        }
    }

    @Override
    public boolean isFinished() {
        if(m_ArmSystem.getArmAngle() > ArmConstants.kArmLowAngle - 0.03 && m_ArmSystem.getArmAngle() < ArmConstants.kArmLowAngle + 0.03 
           && m_ClawSystem.getClawAngle() > ClawConstants.kClawLowAngle - 0.03 && m_ClawSystem.getClawAngle() < ClawConstants.kClawLowAngle + 0.03) {
            return true;
        }
        else {
            return false;
        }
    }
    
    @Override
    public void end(boolean done) {
        m_ArmSystem.stopArm();
        m_ClawSystem.stopClaw();
    }

}