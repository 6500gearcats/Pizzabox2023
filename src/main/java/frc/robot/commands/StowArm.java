package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class StowArm extends CommandBase {
    
    private final Arm m_ArmSystem;
    private final Claw m_ClawSystem;

    public StowArm(Arm theArm, Claw theClaw) {
        m_ArmSystem = theArm;
        m_ClawSystem = theClaw;
    }

    @Override
    public void initialize() {
        m_ArmSystem.armDown();

        if(m_ClawSystem.getClawAngle() > ClawConstants.kClawStowAngle)
            m_ClawSystem.clawDown();
        else
            m_ClawSystem.clawUp();
    }

    @Override
    public void execute() {
        if(m_ArmSystem.LimitSwitchPressed()) {
            m_ArmSystem.stopArm();
        }
        if(m_ClawSystem.getClawAngle() > ClawConstants.kClawStowAngle - ClawConstants.kClawTolerance && m_ClawSystem.getClawAngle() < ClawConstants.kClawStowAngle + ClawConstants.kClawTolerance) {
            m_ClawSystem.stopClawTilt();
        }
    }

    @Override
    public boolean isFinished() {
        return m_ArmSystem.LimitSwitchPressed() && m_ClawSystem.getClawAngle() > ClawConstants.kClawStowAngle - ClawConstants.kClawTolerance && m_ClawSystem.getClawAngle() < ClawConstants.kClawStowAngle + ClawConstants.kClawTolerance;
    }
    
    @Override
    public void end(boolean done) {
        m_ArmSystem.stopArm();
        m_ClawSystem.stopClawTilt();
    }

}
