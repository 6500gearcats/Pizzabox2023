package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmDownWithSpeed extends CommandBase{
    
    private final Arm m_ArmSystem;
    private double m_ArmSpeed;

    public ArmDownWithSpeed(Arm theArm, double speed) {
        m_ArmSystem = theArm;
        m_ArmSpeed = speed;
        addRequirements(m_ArmSystem);
    }

    @Override
    public void execute() {
        m_ArmSystem.armDownSpeed(m_ArmSpeed);
    }

    public void end(boolean done) {
        m_ArmSystem.stopArm();
    }

}
