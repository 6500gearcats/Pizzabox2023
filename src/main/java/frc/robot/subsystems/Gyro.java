package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

    AHRS ahrs;
    Joystick stick;

    public Gyro() {
        // stick = new Joystick(0);
        try {
            /* Communicate w/navX-MXP via the MXP SPI Bus. */
            /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
            /*
             * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
             * details.
             */
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    }

    /**
     * Display navX-MXP Sensor Data on Smart Dashboard
     */
    public void periodic() {

        /* Display 6-axis Processed Angle Data */
        SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
        SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

        /* Display tilt-corrected, Magnetometer-based heading (requires */
        /* magnetometer calibration to be useful) */

        SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

        /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
        SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

        SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
        SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

        SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
        SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

        /* Display estimates of velocity/displacement. Note that these values are */
        /* not expected to be accurate enough for estimating robot position on a */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially */
        /* double (displacement) integration. */

        SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
        SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
        SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

        // /* Display Raw Gyro/Accelerometer/Magnetometer Values */
        // /* NOTE: These values are not normally necessary, but are made available */
        // /* for advanced users. Before using this data, please consider whether */
        // /* the processed data (see above) will suit your needs. */

        // SmartDashboard.putNumber( "RawGyro_X", ahrs.getRawGyroX());
        // SmartDashboard.putNumber( "RawGyro_Y", ahrs.getRawGyroY());
        // SmartDashboard.putNumber( "RawGyro_Z", ahrs.getRawGyroZ());
        // SmartDashboard.putNumber( "RawAccel_X", ahrs.getRawAccelX());
        // SmartDashboard.putNumber( "RawAccel_Y", ahrs.getRawAccelY());
        // SmartDashboard.putNumber( "RawAccel_Z", ahrs.getRawAccelZ());
        // SmartDashboard.putNumber( "RawMag_X", ahrs.getRawMagX());
        // SmartDashboard.putNumber( "RawMag_Y", ahrs.getRawMagY());
        // SmartDashboard.putNumber( "RawMag_Z", ahrs.getRawMagZ());
        // SmartDashboard.putNumber( "IMU_Temp_C", ahrs.getTempC());

        // /* Omnimount Yaw Axis Information */
        // /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
        // AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        // SmartDashboard.putString( "YawAxisDirection", yaw_axis.up ? "Up" : "Down" );
        // SmartDashboard.putNumber( "YawAxis", yaw_axis.board_axis.getValue() );

        // /* Sensor Board Information */
        // SmartDashboard.putString( "FirmwareVersion", ahrs.getFirmwareVersion());

        // /* Quaternion Data */
        // /* Quaternions are fascinating, and are the most compact representation of */
        // /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
        // /* from the Quaternions. If interested in motion processing, knowledge of */
        // /* Quaternions is highly recommended. */
        // SmartDashboard.putNumber( "QuaternionW", ahrs.getQuaternionW());
        // SmartDashboard.putNumber( "QuaternionX", ahrs.getQuaternionX());
        // SmartDashboard.putNumber( "QuaternionY", ahrs.getQuaternionY());
        // SmartDashboard.putNumber( "QuaternionZ", ahrs.getQuaternionZ());

        // /* Connectivity Debugging Support */
        // SmartDashboard.putNumber( "IMU_Byte_Count", ahrs.getByteCount());
        // SmartDashboard.putNumber( "IMU_Update_Count", ahrs.getUpdateCount());

    }

    /* Return the NavX pitch angle */
    public float getPitch() {
        return ahrs.getPitch();
    }

}
