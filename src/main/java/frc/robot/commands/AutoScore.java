package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Gyro;

public class AutoScore extends SequentialCommandGroup {

    Command path1, path2, path3, path4;

    public AutoScore(
            Command path1, Command path2, Command path3, Command path4, 
            DriveSubsystem drive,
            Gyro gyro,
            Arm arm,
            Claw claw) {

        super(
                new CloseClaw(claw).withTimeout(0.5)
                        .andThen(new MoveArmToPosition(ArmConstants.kArmHighAngle, arm)).withTimeout(5.0)
                        .andThen(path1)
                        .andThen(new OpenClaw(claw).withTimeout(0.5))
                        .andThen(path2)
                        .andThen(new MoveArmToPosition(ArmConstants.kArmStowAngle, arm)).withTimeout(5.0)
                        .andThen(path3)
                        .andThen(path4));
    }
}
