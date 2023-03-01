package frc.robot.subsystems;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

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

    private double ArmPosition;
    private boolean lowerLimit;

    public Arm() {
      if (RobotBase.isSimulation()) {
        REVPhysicsSim.getInstance().addSparkMax(m_CanSparkMaxArm, DCMotor.getNEO(1));
      }
        


    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

        //ArmPosition = m_tiltArmEncoder.getAbsolutePosition();
        //lowerLimit = m_lowerLimitSwitch.get();
      
      //SmartDashboard.putNumber("Arm position", ArmPosition);
    }


    //Moves arm up and down
    public void armUp() {
        m_tiltMotor.set(ArmConstants.kArmForwardSpeed);
    }

    public void armDown() {
        m_tiltMotor.set(ArmConstants.kArmReverseSpeed);
    }

    public boolean ArmAtAngle() {
      double ArmAngle = m_tiltArmEncoder.getAbsolutePosition();
      boolean mbArmAtAngle = ArmAngle > ArmConstants.kEncoderUpperThreshold;
      return mbArmAtAngle;
    }

    public boolean LowSwitchPressed() {
        return lowerLimit;
    }

    public double getArmAngle() {
        return m_tiltArmEncoder.getAbsolutePosition();
    }

    public void stopArm() {
        // Stop the arm motor moving.
        m_tiltMotor.stopMotor();
      }
}