package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;

public class ReadyForPickUp extends ParallelCommandGroup{
    public ReadyForPickUp(Arm arm, Claw claw){
        addCommands(
            new MoveArmToPosition(ArmConstants.kLoadingStation, arm),
            new RunCommand(()-> claw.moveTo(ClawConstants.kClawStowAngle))
        );
    }
}
