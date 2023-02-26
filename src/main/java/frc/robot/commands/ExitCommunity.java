// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ExitCommunity extends CommandBase {
  private final DriveSubsystem m_robotDrive;

public ExitCommunity(DriveSubsystem robotDrive){
  m_robotDrive = robotDrive;
  addRequirements(m_robotDrive);
}

public void execute()
{
  getExitCommunity();
}

public Command getExitCommunity(){
  TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

// Trajectory from grid to outside community area
Trajectory exitCommunity = TrajectoryGenerator.generateTrajectory(
  // Start at the origin facing the +X direction
  new Pose2d(0, 0, new Rotation2d(0)),
  // Pass through these two interior waypoints, making an 's' curve path
  List.of(new Translation2d(0, 1)),
  // End 3 meters straight ahead of where we started, facing forward
  new Pose2d(0, AutoConstants.exitCommunityEnd, new Rotation2d(0)),
  config);
    ///new Pose2d(0, AutoConstants.exitCommunityEnd, new Rotation2d(0)),

ProfiledPIDController thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    exitCommunity,
    m_robotDrive::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,
    // Position controllers
    new PIDController(AutoConstants.kPXController, 0, 0),
    new PIDController(AutoConstants.kPYController, 0, 0),
    thetaController,
    m_robotDrive::setModuleStates,
    m_robotDrive);

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(exitCommunity.getInitialPose());

// Run path following command, then stop at the end.
return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
}
}


