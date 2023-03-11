package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.*;

public class AutoScore extends SequentialCommandGroup{
    public AutoScore(DriveSubsystem driveSubsystem, Arm arm, Claw claw, Gyro gyro) {
        addCommands(
            new CloseClaw(claw),
            new MoveArmToPosition(ArmConstants.kArmHighAngle, arm),
            //new RunCommand(() -> driveSubsystem.drive(0.3, 0, 0, false)),
            new OpenClaw(claw),
            new MoveArmToPosition(ArmConstants.kArmStowAngle, arm),
            new RunCommand(() -> driveSubsystem.drive(0, 0, 180, false))
        );
    }
}
