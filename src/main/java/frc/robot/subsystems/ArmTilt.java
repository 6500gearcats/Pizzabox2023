package frc.robot.subsystems;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmTiltConstants;

public class ArmTilt extends SubsystemBase {

    // Create the arm tilter motor and claw tilter motor
    // The constants are not corect right now, will be replaced.

    private final MotorController m_tilterMotor = new CANSparkMax(ArmTiltConstants.kTiltMotorPort, MotorType.kBrushed);
    private final MotorController m_clawTilterMotor = new CANSparkMax(ArmTiltConstants.kClawTiltMotorPort, MotorType.kBrushed);

    private DutyCycleEncoder m_tiltArmEncoder;

    private SparkMaxLimitSwitch m_lowerLimitSwitch;

    public ArmTilt() {}

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    
    public void angleClawDown() {
      // Quickly lower the claw.
      m_clawTilterMotor.set(ArmTiltConstants.kClawTiltReverseSpeed);
    }

    public void setArmReverse() {
      // Quickly reverse the tilt of the arm after depositing object.
      m_tilterMotor.set(ArmTiltConstants.kTiltReverseSpeed);
    }

    public void angleClawUp() {
      // Tilt claw up quickly.
    m_clawTilterMotor.set(ArmTiltConstants.kClawTiltForwardSpeed);
  }

    public void setArmForward() {
        // Tilt arm quickly forward.
      m_tilterMotor.set(ArmTiltConstants.kTiltForwardSpeed);
    }

    // NOTICE!!! The following three methods may not be needed.

    public void stowArm() {
        //Slowly reverse the tilt of the arm after arm nears robot.
      m_tilterMotor.set(ArmTiltConstants.kTiltStowSpeed);
    }

    public void angleClawDownSlow() {
      //Slowly do angleClawDown
    m_tilterMotor.set(ArmTiltConstants.kTiltStowSpeed);
  }
    public void setSecondTierSpeed() {
        // Tilt the arm slowly to deposit objects on the second teir.
      m_tilterMotor.set(ArmTiltConstants.kSecondTierSpeed);
    }

    public void stopArm() {
        // Stop the arm motor moving.
        m_tilterMotor.stopMotor();
      }

      public void stopClaw() {
        // Stop the claw motor moving.
        m_clawTilterMotor.stopMotor();
      }

}
