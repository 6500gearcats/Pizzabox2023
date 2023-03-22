// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.LayoutStyle;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
  public boolean turboEnable = false;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private AHRS m_gyro;

  private int m_gyroSim;
  private SimDouble m_simAngle;
  private SimBoolean m_connected;
  private SimBoolean m_calibrating;
  private boolean m_fieldOriented;

  private ChassisSpeeds m_lastSpeeds;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  private final Field2d m_field = new Field2d();

  private Pose2d m_simOdometryPose;

  private final Pose2d startPositions[] = {
    new Pose2d(2.83, 4.70, new Rotation2d()), 
    new Pose2d(2.45, 2.75, new Rotation2d()),  
    new Pose2d(2.99, 1.05, new Rotation2d())
  };

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      m_gyro = new AHRS(SPI.Port.kMXP);
      System.out.println("AHRS constructed");
    } catch (RuntimeException ex) {
      System.out.println("AHRS not constructed");
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    m_gyro.setAngleAdjustment(180.0);
    m_gyro.zeroYaw();

    if (RobotBase.isSimulation()) {
      m_gyroSim  = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      m_simAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_gyroSim, "Yaw"));
      m_connected = new SimBoolean(SimDeviceDataJNI.getSimValueHandle(m_gyroSim, "Connected"));
      m_calibrating = new SimBoolean(SimDeviceDataJNI.getSimValueHandle(m_gyroSim, "Calibrating"));
      m_connected.set(true);
      m_calibrating.set(false);
      SmartDashboard.putNumber(getName(), getPitch());
    }

       
    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }, startPositions[DriverStation.getLocation()-1]);

        m_lastSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0,0.0, 0.0, Rotation2d.fromDegrees(0.0));

    m_simOdometryPose = m_odometry.getPoseMeters();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

    if (Robot.isReal()) {
      m_field.setRobotPose(m_odometry.getPoseMeters());
    }
    else{
      m_field.setRobotPose(m_simOdometryPose);
    }

    SmartDashboard.putNumber("NavX Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("NavX Yaw angle", getAngle());

    SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);

  }

  @Override
  public void simulationPeriodic() {
    // Update the odometry in the periodic block
    REVPhysicsSim.getInstance().run();

    //angle.set(5.0);
    double angle = getPose().getRotation().getDegrees();
    SmartDashboard.putNumber("SimAngle", angle);
    m_simAngle.set(angle);


  }

  /** 
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    if (Robot.isReal()) {
      return m_odometry.getPoseMeters();
    } else {
      return m_simOdometryPose;
    }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

        m_simOdometryPose = pose;
  }

  /**
   * Updates the odometry of the robot using the swerve module states and the gyro
   * reading. Should
   * be run in periodic() or during every code loop to maintain accuracy.
   */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    if (Robot.isSimulation()) {
      SwerveModuleState[] measuredStates =
          new SwerveModuleState[] {
          m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
      };
      //ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(measuredStates);
      ChassisSpeeds speeds = m_lastSpeeds;

      Twist2d twist = new Twist2d(
        speeds.vxMetersPerSecond * .02,
        speeds.vyMetersPerSecond * .02,
        speeds.omegaRadiansPerSecond * .02);

      m_simOdometryPose =
          m_simOdometryPose.exp(twist);

      SmartDashboard.putNumber("new x", twist.dx );
      SmartDashboard.putNumber("new y ", twist.dy );
      SmartDashboard.putNumber("new theta ", twist.dtheta );

    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    m_fieldOriented = fieldRelative;
    // Adjust input based on max speed
    
    rot *= DriveConstants.kMaxAngularSpeed;
    // Non linear speed set
    xSpeed = Math.signum(xSpeed)*Math.abs(Math.pow(xSpeed,2));
    ySpeed = Math.signum(ySpeed)*Math.abs(Math.pow(ySpeed,2));

    // xSpeed *= DriveConstants.kNormalSpeedMetersPerSecond;
    // ySpeed *= DriveConstants.kNormalSpeedMetersPerSecond;

    if(turboEnable)
    {
      xSpeed *= DriveConstants.kTurboModeModifier;
      ySpeed *= DriveConstants.kTurboModeModifier;
      System.out.println("here" + xSpeed + ySpeed);
    }
  
    m_lastSpeeds =  (fieldRelative) ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getAngle()))
                                    : new ChassisSpeeds(xSpeed, ySpeed, rot);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_lastSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /* Return the NavX pitch angle */
  public double getPitch() {
    return m_gyro.getPitch();
  }

  /* Return the NavX yaw angle */
  public double getAngle() {
    //return -m_gyro.getYaw();
    return -m_gyro.getAngle();
  }


  public boolean toggleFieldOriented() {
    m_fieldOriented = !m_fieldOriented;
    return m_fieldOriented;
  }

}
