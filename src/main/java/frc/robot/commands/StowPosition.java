package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class StowPosition extends CommandBase {
    
    private final Arm m_ArmSystem;
    private final Claw m_ClawSystem;

    public StowPosition(Arm theArm, Claw theClaw) {
        m_ArmSystem = theArm;
        m_ClawSystem = theClaw;
    }

    @Override
    public void initialize() {
        if(m_ArmSystem.getArmAngle() > ArmConstants.kArmStowAngle && m_ClawSystem.getClawAngle() > ClawConstants.kClawStowAngle) {
            m_ArmSystem.armDown();
            m_ClawSystem.clawDown();
        }
        else if(m_ArmSystem.getArmAngle() > ArmConstants.kArmStowAngle && m_ClawSystem.getClawAngle() < ClawConstants.kClawStowAngle){
            m_ArmSystem.armDown();
            m_ClawSystem.clawUp();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmStowAngle && m_ClawSystem.getClawAngle() > ClawConstants.kClawStowAngle) {
            m_ArmSystem.armUp();
            m_ClawSystem.clawDown();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmStowAngle && m_ClawSystem.getClawAngle() < ClawConstants.kClawStowAngle) {
            m_ArmSystem.armUp();
            m_ClawSystem.clawUp();
        }
    }

    @Override
    public boolean isFinished() {
        if(m_ArmSystem.getArmAngle() > ArmConstants.kArmStowAngle - 0.3 && m_ArmSystem.getArmAngle() < ArmConstants.kArmStowAngle + 0.3 
           && m_ClawSystem.getClawAngle() > ClawConstants.kClawStowAngle - 0.3 && m_ClawSystem.getClawAngle() < ClawConstants.kClawStowAngle + 0.3) {
            return true;
        }
        else {
            return false;
        }
    }
    
    @Override
    public void end(boolean done) {
        m_ArmSystem.stopArm();
        m_ClawSystem.stopClawTilt();
    }

}