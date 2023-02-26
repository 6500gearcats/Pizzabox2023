package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmDown extends CommandBase{
    
    private final Arm m_ArmSystem;

    public ArmDown(Arm theArm) {
        m_ArmSystem = theArm;
    }

    @Override
    public void execute() {
        m_ArmSystem.armDown();
    }

}
