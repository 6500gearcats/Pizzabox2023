// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmUp;
import frc.robot.commands.ClawDown;
import frc.robot.commands.ClawUp;
import frc.robot.commands.ClimbPlatform;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.FloorPosition;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.ScoreHighPosition;
import frc.robot.commands.StowPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm m_Arm = new Arm();
  private final Claw m_Claw = new Claw();
  private final Gyro m_Gyro = new Gyro();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.06), //0.1
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.06), //0.1
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    //DRIVER CONTROLLER
    //while left button is pressed, speed is modified by the slow mode modifier constant (currently 3/7)
    new JoystickButton(m_driverController, Button.kLeftBumper.value).whileTrue(new DriveSlow(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onFalse(new DriveNormal(m_robotDrive));

    //GUNNER CONTROLLER
    //sets the left stick to move arm up, increasing in speed with how far the joystick is pushed
    //new Trigger(() -> m_gunnerController.getLeftY() > 0).whileTrue(new ArmUpWithSpeed(m_Arm, (ArmConstants.kArmForwardMaxSpeed * m_gunnerController.getLeftY())));
    //sets the left stick to move arm down, increasing in speed with how far the joystick is pushed
    //new Trigger(() -> m_gunnerController.getLeftY() < 0).whileTrue(new ArmDownWithSpeed(m_Arm, (ArmConstants.kArmReverseMaxSpeed * m_gunnerController.getLeftY())));
  
    //sets left stick to arm up or down at constant speed
    new Trigger(() -> m_gunnerController.getLeftY() < -0.05).whileTrue(new ArmUp(m_Arm));
    new Trigger(() -> m_gunnerController.getLeftY() >0.05).whileTrue(new ArmDown(m_Arm));

    //sets the right stick to move claw up, at a constand speed
    new Trigger(() -> m_gunnerController.getRightY() > 0.05).whileTrue(new ClawUp(m_Claw));
    //sets the right stick to move claw down, at a constant speed
    new Trigger(() -> m_gunnerController.getRightY() < -0.05).whileTrue(new ClawDown(m_Claw));
    //sets claw open to right button and claw close to left button
    new JoystickButton(m_gunnerController, Button.kRightBumper.value).onTrue(new OpenClaw(m_Claw));
    new JoystickButton(m_gunnerController, Button.kLeftBumper.value).onTrue(new CloseClaw(m_Claw));
    //sets stow arm to back button
    new JoystickButton(m_gunnerController, Button.kBack.value).whileTrue(new StowArm(m_Arm, m_Claw));
    //sets arm to floor to start button
    new JoystickButton(m_gunnerController, Button.kStart.value).whileTrue(new ToFloor(m_Arm, m_Claw));
    //sets Score high to y button
    new JoystickButton(m_gunnerController, Button.kY.value).whileTrue(new MoveArmToPosition(ArmConstants.kArmHighAngle, m_Arm));
    //sets score mid to b button
    new JoystickButton(m_gunnerController, Button.kB.value).whileTrue(new MoveArmToPosition(ArmConstants.kArmMidAngle, m_Arm));
    //sets score low to a button
    new JoystickButton(m_gunnerController, Button.kA.value).whileTrue(new MoveArmToPosition(ArmConstants.kArmLowAngle, m_Arm));
    //stops arm and claw with x
    new JoystickButton(m_gunnerController, Button.kX.value).whileTrue(new StopArm(m_Arm, m_Claw));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("fullAuto", new PathConstraints(4, 3));

    // This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        m_robotDrive::getPose, // Pose2d supplier
        m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        DriveConstants.kDriveKinematics,
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        m_robotDrive::setModuleStates,// Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    Command fullAuto = autoBuilder.fullAuto(pathGroup);

    // Run path following command, then stop at the end.
    return fullAuto.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
