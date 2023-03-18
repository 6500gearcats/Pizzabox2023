package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmDownWithSpeed extends CommandBase{
    
    private final Arm m_ArmSystem;
    private double m_ArmSpeed;
    private Supplier<Boolean> m_SlowMode;

    public ArmDownWithSpeed(Arm theArm, Supplier<Boolean> slowMode) {
        m_ArmSystem = theArm;
        m_SlowMode = slowMode;
        addRequirements(m_ArmSystem);
    }

    @Override
    public void execute() {
        if(m_SlowMode.get()) {
            m_ArmSpeed = ArmConstants.kArmReverseSpeed * ArmConstants.kArmSlowModifier;
        }
        else {
            m_ArmSpeed = ArmConstants.kArmReverseSpeed;
        }
        m_ArmSystem.armDownSpeed(m_ArmSpeed);
    }

    public void end(boolean done) {
        m_ArmSystem.stopArm();
    }

}
