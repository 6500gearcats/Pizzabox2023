package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class StopArm extends CommandBase {
    
    private final Arm m_ArmSystem;
    private final Claw m_ClawSystem;

    public StopArm(Arm theArm, Claw theClaw) {
        m_ArmSystem = theArm;
        m_ClawSystem = theClaw;
    }

    @Override
    public void initialize() {
        m_ArmSystem.stopArm();
        m_ClawSystem.stopClawTilt();
    }
}