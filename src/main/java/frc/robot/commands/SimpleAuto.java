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

public class SimpleAuto extends CommandBase {
  private final DriveSubsystem m_robotDrive;

public SimpleAuto(DriveSubsystem robotDrive){
  m_robotDrive = robotDrive;
}
public Command getAutoCommand(){
  TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

// Trajectory from start to the grid
  Trajectory toGrid = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    null, 
    // End .85 meters straight ahead of where we started, facing forward
    new Pose2d(0, 0.85, new Rotation2d(0)),
    config);

  Trajectory exitCommunity = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0.85, new Rotation2d(0)),
    null,
    // End 5.15 meters behind the 2nd position, facing forward
    new Pose2d(0, -5.15, new Rotation2d(0)),
    config);

ProfiledPIDController thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    toGrid,
    m_robotDrive::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,
    // Position controllers
    new PIDController(AutoConstants.kPXController, 0, 0),
    new PIDController(AutoConstants.kPYController, 0, 0),
    thetaController,
    m_robotDrive::setModuleStates,
    m_robotDrive);

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(toGrid.getInitialPose());

// Run path following command, then stop at the end.
return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
}
}


