package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmUp extends CommandBase{
    
    private final Arm m_ArmSystem;

    public ArmUp(Arm theArm) {
        m_ArmSystem = theArm;
    }

    @Override
    public void execute() {
        m_ArmSystem.armUp();
    }

}