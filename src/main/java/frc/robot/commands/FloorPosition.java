package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class FloorPosition extends CommandBase {
    
    private final Arm m_ArmSystem;
    private final Claw m_ClawSystem;

    public FloorPosition(Arm theArm, Claw theClaw) {
        m_ArmSystem = theArm;
        m_ClawSystem = theClaw;
    }

    @Override
    public void initialize() {
        if(m_ArmSystem.getArmAngle() > ArmConstants.kArmFloorAngle && m_ClawSystem.getClawAngle() > ArmConstants.kClawFloorAngle) {
            m_ArmSystem.armDown();
            m_ClawSystem.clawDown();
        }
        else if(m_ArmSystem.getArmAngle() > ArmConstants.kArmFloorAngle && m_ClawSystem.getClawAngle() < ArmConstants.kClawFloorAngle){
            m_ArmSystem.armDown();
            m_ClawSystem.clawUp();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmFloorAngle && m_ClawSystem.getClawAngle() > ArmConstants.kClawFloorAngle) {
            m_ArmSystem.armUp();
            m_ClawSystem.clawDown();
        }
        else if(m_ArmSystem.getArmAngle() < ArmConstants.kArmFloorAngle && m_ClawSystem.getClawAngle() < ArmConstants.kClawFloorAngle) {
            m_ArmSystem.armUp();
            m_ClawSystem.clawUp();
        }
    }

    @Override
    public boolean isFinished() {
        if(m_ArmSystem.getArmAngle() == ArmConstants.kArmFloorAngle && m_ClawSystem.getClawAngle() == ArmConstants.kClawFloorAngle) {
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