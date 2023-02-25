package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TiltArmDown extends CommandBase{
    
    private final Arm m_armSystem;

    public TiltArmDown(Arm theArm) {
        m_armSystem = theArm;
        addRequirements(m_armSystem);
    }

    @Override
    public void initialize() {
        m_armSystem.stowArm();
    }


    // The arm will stop if raised too high ...
    // HOPEFULLY
    @Override
    public boolean isFinished() {
        return m_armSystem.ArmIsFullyRetracted();
    }    

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopArm();
    }

}

