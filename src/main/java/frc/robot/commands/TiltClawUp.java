package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TiltClawUp extends CommandBase {

    private final Arm m_clawSystem;

    public TiltClawUp(Arm theClaw) {
        m_clawSystem = theClaw;
        addRequirements(m_clawSystem);
    }

    @Override
    public void initialize() {
        m_clawSystem.angleClawUpSlow();
    }


    // The arm will stop if raised too high ...
    // HOPEFULLY
    @Override
    public boolean isFinished() {
        return m_clawSystem.ArmAngle();
    }    

    @Override
    public void end(boolean interrupted) {
        m_clawSystem.stopArm();
    }

}
