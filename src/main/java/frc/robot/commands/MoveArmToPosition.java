package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

/**
 * Sequential command that encapsulates the various sub-commands used
 * by the robot in climbing a bar in the climbing challenge
 */
public class MoveArmToPosition extends SequentialCommandGroup {
    /**
     * Creates a new ClimbBar.
     * 
     * @param climber The climber subsystem this command will run on
     */

     Arm m_arm;
    public MoveArmToPosition(double position, Arm arm) {
        m_arm = arm;

        addCommands(
            // Setup the robot for climbing the bar
            new FunctionalCommand(
                () -> m_arm.stopArm(),
                () -> m_arm.moveToTarget(position),
                interrupted -> m_arm.stopArm(),
                () -> m_arm.armAtTarget(position),
                m_arm));
    }
}