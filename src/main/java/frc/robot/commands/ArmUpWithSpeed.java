package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmUpWithSpeed extends CommandBase{
    
    private final Arm m_ArmSystem;
    private double m_ArmSpeed;
    private Supplier<Boolean> m_SlowMode;

    public ArmUpWithSpeed(Arm theArm, Supplier<Boolean> slowMode) {
        m_ArmSystem = theArm;
        m_SlowMode = slowMode;
        addRequirements(m_ArmSystem);
    }

    @Override
    public void execute() {
        if(m_SlowMode.get()) {
            m_ArmSpeed = ArmConstants.kArmForwardSpeed * ArmConstants.kArmSlowModifier;
        }
        else {
            m_ArmSpeed = ArmConstants.kArmForwardSpeed;
        }
        m_ArmSystem.armUpSpeed(m_ArmSpeed);
    }

    @Override
    public void end(boolean done) {
        m_ArmSystem.stopArm();
    }

}