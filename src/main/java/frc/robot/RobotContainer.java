// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints; import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1), //0.1
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1), //0.1
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1),
                !m_driverController.getRightBumper()),
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

    //DRIVER CONTROLLER
    //while left button is pressed, speed is modified by the turbo mode modifier constant 
    new JoystickButton(m_driverController, Button.kLeftBumper.value).whileTrue(new DriveTurbo(m_robotDrive));
    new JoystickButton(m_driverController, Button.kLeftBumper.value).onFalse(new DriveNormal(m_robotDrive));
    //Turn on lights: Yellow = Back,     Purple = Start
    new JoystickButton(m_driverController, Button.kBack.value).whileTrue(new LightYellow());
    new JoystickButton(m_driverController, Button.kStart.value).whileTrue(new LightPurple());

    // Use the automated Platform climb      
    new JoystickButton(m_driverController, Button.kA.value).onTrue(new ClimbPlatform(m_robotDrive));

    // Set the wheels in locked arrangement to prevent movement
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));


    //GUNNER CONTROLLER
    //sets the left stick to move arm up, increasing in speed with how far the joystick is pushed
    //new Trigger(() -> m_gunnerController.getLeftY() > 0).whileTrue(new ArmUpWithSpeed(m_Arm, (ArmConstants.kArmForwardMaxSpeed * m_gunnerController.getLeftY())));
    //sets the left stick to move arm down, increasing in speed with how far the joystick is pushed
    //new Trigger(() -> m_gunnerController.getLeftY() < 0).whileTrue(new ArmDownWithSpeed(m_Arm, (ArmConstants.kArmReverseMaxSpeed * m_gunnerController.getLeftY())));

    //Makes the arm move slow when the left bumper is pressed
    new Trigger(() -> (m_gunnerController.getLeftTriggerAxis() > 0.5))
      .onTrue(new ArmSlow(m_Arm));
    new Trigger(() -> (m_gunnerController.getLeftTriggerAxis() > 0.5))
      .onFalse(new ArmNormal(m_Arm));
    //Makes the claw move slaw when the right bumper is pressed
    new Trigger(() -> (m_gunnerController.getRightTriggerAxis() > 0.5))
      .onTrue(new ClawSlow(m_Claw));
    new Trigger(() -> (m_gunnerController.getRightTriggerAxis() > 0.5))
      .onFalse(new ClawNormal(m_Claw));
  
    //sets left stick to arm up or down at constant speed
    new Trigger(() -> m_gunnerController.getLeftY() < -0.05).whileTrue(new ArmUpWithSpeed(m_Arm, -0.5));
    //new RunCommand(
    //        () -> (m_driverController.getLeftTriggerAxis() > 0.5 ?
    //        m_Arm.armUpSpeed(m_driverController.getLeftTriggerAxis()) :
    //        m_Arm.armUpSpeed(m_driverController.getLeftTriggerAxis()),
    //        m_Arm);
    new Trigger(() -> m_gunnerController.getLeftY() >0.05).whileTrue(new ArmDownWithSpeed(m_Arm, 0.5));

    //sets the right stick to move claw up at a constand speed
    new Trigger(() -> m_gunnerController.getRightY() > 0.05).whileTrue(new ClawUpWithSpeed(m_Claw, -0.2));
    //sets the right stick to move claw down, at a constant speed
    new Trigger(() -> m_gunnerController.getRightY() < -0.05).whileTrue(new ClawDownWithSpeed(m_Claw, 0.2));
    //sets claw open to right button and claw close to left button
    new JoystickButton(m_gunnerController, Button.kRightBumper.value).onTrue(new CloseClaw(m_Claw));
    new JoystickButton(m_gunnerController, Button.kLeftBumper.value).onTrue(new OpenClaw(m_Claw));
    //stops arm and claw with back button
    new JoystickButton(m_gunnerController, Button.kBack.value).whileTrue(new StopArm(m_Arm, m_Claw));
    //sets arm to floor to start button
    new JoystickButton(m_gunnerController, Button.kStart.value).whileTrue(new ToFloor(m_Arm, m_Claw));
    //sets Score high to y button
    new JoystickButton(m_gunnerController, Button.kY.value).whileTrue(new MoveArmToPosition(ArmConstants.kArmHighAngle, m_Arm));
    //sets score mid to b button
    new JoystickButton(m_gunnerController, Button.kB.value).whileTrue(new MoveArmToPosition(ArmConstants.kArmMidAngle, m_Arm));
    //sets score low to a button
    new JoystickButton(m_gunnerController, Button.kA.value).whileTrue(new ReadyForPickUp(m_Arm, m_Claw));
    //sets stow arm to x button
    new JoystickButton(m_gunnerController, Button.kX.value).whileTrue(new StowArm(m_Arm, m_Claw));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    //ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("fullAuto", new PathConstraints(4, 3));

    ArrayList<PathPlannerTrajectory> pathGroup1 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Path Start 1", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup2 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Path Start 2", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup3 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Path End 3", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup4 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Path End 1", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup5 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 1_1", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup6 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 1_2", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup7 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 1_3", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup8 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 2_1", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup9 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 2_2", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup10 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 2_3", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup11 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 3_1", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup12 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 3_2", new PathConstraints(4, 3));
    ArrayList<PathPlannerTrajectory> pathGroup13 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Cube Place 3_3", new PathConstraints(4, 3));
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

    //Command fullAuto = autoBuilder.fullAuto(pathGroup);

    // Command pathStart1 = autoBuilder.fullAuto(pathGroup1);
    // Command pathStart2 = autoBuilder.fullAuto(pathGroup2);
    Command pathEnd3 = autoBuilder.fullAuto(pathGroup3);
    Command pathEnd1 = autoBuilder.fullAuto(pathGroup4);
    Command cubePath1_1 = autoBuilder.fullAuto(pathGroup5);
    Command cubePath1_2 = autoBuilder.fullAuto(pathGroup6);
    Command cubePath1_3 = autoBuilder.fullAuto(pathGroup7);
    Command cubePath2_1 = autoBuilder.fullAuto(pathGroup8);
    Command cubePath2_2 = autoBuilder.fullAuto(pathGroup9);
    Command cubePath2_3 = autoBuilder.fullAuto(pathGroup10);
    Command cubePath3_1 = autoBuilder.fullAuto(pathGroup11);
    Command cubePath3_2 = autoBuilder.fullAuto(pathGroup12);
    Command cubePath3_3 = autoBuilder.fullAuto(pathGroup13);

    // Run path following command, then stop at the end.
    //return fullAuto.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    switch (DriverStation.getLocation()) {
      case 1:
      {
        System.out.println("Starting Path 1");
        return new CloseClaw(m_Claw).withTimeout(0.5)
        .andThen(new MoveArmToPosition(ArmConstants.kArmHighAngle, m_Arm).withTimeout(5.0)
        .andThen(cubePath1_1)
        .andThen(new OpenClaw(m_Claw).withTimeout(0.5))
        .andThen(cubePath1_2)
        .andThen(new StowArm(m_Arm, m_Claw)).withTimeout(5.0))
        .andThen(cubePath1_3)
        .andThen(pathEnd1)
        .andThen(()-> m_robotDrive.drive(0, 0, 0, false));       
      }
      case 2:
      {
        System.out.println("Starting Path 2");
        return new CloseClaw(m_Claw).withTimeout(0.5)
        .andThen(new MoveArmToPosition(ArmConstants.kArmHighAngle, m_Arm)).withTimeout(5.0)
        .andThen(cubePath2_1)
        .andThen(new OpenClaw(m_Claw).withTimeout(0.5))      
        .andThen(cubePath2_2)
        .andThen(new StowArm(m_Arm, m_Claw).withTimeout(5.0))
        .andThen(cubePath2_3)
        .andThen(new ClimbPlatform(m_robotDrive))
        .andThen(()-> m_robotDrive.drive(0, 0, 0, false));
      }
      case 3:
      {
        System.out.println("Starting Path 3");
        return new CloseClaw(m_Claw).withTimeout(0.5)
        .andThen(new MoveArmToPosition(ArmConstants.kArmHighAngle, m_Arm)).withTimeout(5.0)
        .andThen(cubePath3_1)
        .andThen(new OpenClaw(m_Claw).withTimeout(0.5))
        .andThen(cubePath3_2)
        .andThen(new StowArm(m_Arm, m_Claw).withTimeout(5.0))
        .andThen(cubePath3_3)
        .andThen(pathEnd3)
        .andThen(()-> m_robotDrive.drive(0, 0, 0, false));
      }
      default:
      {
        return new WaitCommand(0);
      }
    }
  }
}
