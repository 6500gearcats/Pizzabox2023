package frc.robot.subsystems;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    public boolean slowArm;

    // Create the arm tilter motor and claw tilter motor
    // The constants are not corect right now, will be replaced.

    private final CANSparkMax m_CanSparkMaxArm = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless); 
    private final MotorController m_tiltMotor =  m_CanSparkMaxArm;

    // Sets upper and lower limit
    private final DutyCycleEncoder m_tiltArmEncoder = new DutyCycleEncoder(0);

    // Sets lower limit
    //private SparkMaxLimitSwitch m_lowerLimitSwitch; 
    private final DigitalInput m_lowerLimitSwitch = new DigitalInput(2);

    //private boolean m_isArmStored = false;
    //private boolean m_isArmCapped = false;

    private final SlewRateLimiter armFilter = new SlewRateLimiter(0.6);

    private double ArmPosition;
    private boolean lowerLimit;

    public Arm() {
      if (RobotBase.isSimulation()) {
        REVPhysicsSim.getInstance().addSparkMax(m_CanSparkMaxArm, DCMotor.getNEO(1));
      }
      
      //m_tiltArmEncoder.reset();

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

        ArmPosition = m_tiltArmEncoder.getAbsolutePosition();
        //lowerLimit = m_lowerLimitSwitch.get();
      
        SmartDashboard.putNumber("Arm Encoder:", ArmPosition);
        SmartDashboard.putNumber("Arm motor speed", m_tiltMotor.get());
        SmartDashboard.putBoolean("Arm limit: ", m_lowerLimitSwitch.get());
    }


    //Moves arm up at constant speed
    public void armUp() {
        if ( AtMaxHeight() ) {
            m_tiltMotor.set(0);
        } else {
            m_tiltMotor.set(ArmConstants.kArmForwardSpeed);
        }
    }

    //same method that takes in a speed to be used instead of our constant, useful in the ArmUp command
    public void armUpSpeed(double speed) {
        if (slowArm) speed *= ArmConstants.kArmSlowModifier;
        if ( AtMaxHeight() ) {
            m_tiltMotor.set(0);
        } else {
            m_tiltMotor.set(armFilter.calculate(speed));
        }
    }

    //Moves arm down at constant speed
    public void armDown() {
        if ( LimitSwitchPressed() ) {
            m_tiltMotor.set(0);
        }
        else if (m_tiltArmEncoder.get() >= 0.75){
          m_tiltMotor.set(ArmConstants.kArmReverseSpeed*ArmConstants.kArmSlowModifier);
        }
        else {
            m_tiltMotor.set(ArmConstants.kArmReverseSpeed);
        }
    }

    //same method that takes in a speed to be used instead of our constant, useful in the ArmDown command
    public void armDownSpeed(double speed) {
        if (slowArm) speed *= ArmConstants.kArmSlowModifier;
        if ( LimitSwitchPressed() ) {
            m_tiltMotor.set(0);
        } else {
            m_tiltMotor.set(speed);
        }
    }

    //returns true if lower limit switch is pressed
    public boolean LimitSwitchPressed() {
      return !m_lowerLimitSwitch.get();
    }

    public boolean AtMaxHeight() {
        return m_tiltArmEncoder.getAbsolutePosition() < ArmConstants.kEncoderUpperThreshold;
    }
    public double getArmAngle() {
        return m_tiltArmEncoder.getAbsolutePosition();
    }

    public void moveToTarget(double target) {
      if (m_tiltArmEncoder.getAbsolutePosition() < target) {
        armDown();
      }
      
      if (m_tiltArmEncoder.getAbsolutePosition() > target) {
        armUp();
      }
    }

    public void stopArm() {
        // Stop the arm motor moving.
        m_tiltMotor.stopMotor();
        if(ArmPosition > ArmConstants.kArmStowAngle){
          armUpSpeed(0.05);
        }

      }

    public void resetFilter() {
      armFilter.reset(0);
    }

    public boolean armAtTarget(double targetAngle) {
      return Math.abs(getArmAngle() - targetAngle) < 0.01;
    }
}
