package frc.robot.subsystems;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmTiltConstants;

public class Arm extends SubsystemBase {

    // Create the arm tilter motor and claw tilter motor
    // The constants are not corect right now, will be replaced.

    private final MotorController m_tilterMotor = new CANSparkMax(ArmTiltConstants.kTiltMotorPort, MotorType.kBrushed);
    private final MotorController m_clawTilterMotor = new CANSparkMax(ArmTiltConstants.kClawTiltMotorPort, MotorType.kBrushed);

    // Sets upper and lower limit
    /* There is some problem with the DutyCycleEncoder definition. It works
    fine in the code, but returns null when tested. */
    private DutyCycleEncoder m_tiltArmEncoder;

    // Sets lower limit
    private SparkMaxLimitSwitch m_lowerLimitSwitch;

    private boolean m_isArmStored = false;
    private boolean m_isArmCapped = false;

    public Arm() {}

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

      double ClawAngle = m_tiltArmEncoder.getAbsolutePosition();
      double ArmPosition = m_tiltArmEncoder.getAbsolutePosition();
      boolean lowerLimit = m_lowerLimitSwitch.isPressed();
      
      SmartDashboard.putNumber("Arm position", ArmPosition);
      SmartDashboard.putNumber("Claw position", ClawAngle);
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

      private boolean m_PastRotationLimit = false;

    public void angleClawDown() {
      // Quickly lower the claw.
      m_clawTilterMotor.set(ArmTiltConstants.kClawTiltReverseSpeed);
    }

    public void setArmReverse() {
      // Quickly reverse the tilt of the arm after depositing object.
      m_tilterMotor.set(ArmTiltConstants.kTiltReverseSpeed);
    }
    
    // Check if the arm has been retracted
    public boolean ArmIsFullyRetracted() {
      boolean lowerLimit = m_lowerLimitSwitch.isPressed();
      SmartDashboard.putBoolean("Lower Limit", lowerLimit);
      return lowerLimit;
    }

    public void angleClawUp() {
      // Tilt claw up quickly.
    m_clawTilterMotor.set(ArmTiltConstants.kClawTiltForwardSpeed);
  }

    public void setArmForward() {
        // Tilt arm quickly forward.
      m_tilterMotor.set(ArmTiltConstants.kTiltForwardSpeed);
    }

    public boolean ArmAngle() {
      double ArmAngle = m_tiltArmEncoder.getAbsolutePosition();
      boolean mbArmAtAngle = ArmAngle > ArmTiltConstants.kEncoderUpperThreshold;
      return mbArmAtAngle;
    }

    public boolean CheckClawAngle() {
      double ClawAngle = m_tiltArmEncoder.getAbsolutePosition();
      boolean mbClawAtAngle = ClawAngle > ArmTiltConstants.kEncoderUpperThreshold;
      return mbClawAtAngle;
    }

    // NOTICE!!! The following methods may not be needed.

    public void stowArm() {
        //Slowly reverse the tilt of the arm after arm nears robot.
      m_tilterMotor.set(ArmTiltConstants.kTiltStowSpeed);
    }

    public void angleClawDownSlow() {
      //Slowly do angleClawDown
    m_clawTilterMotor.set(ArmTiltConstants.kTiltStowSpeed);
  }
    public void setSecondTierSpeed() {
        // Tilt the arm slowly to deposit objects on the second teir.
      m_tilterMotor.set(ArmTiltConstants.kSecondTierSpeed);
    }

    public void angleClawUpSlow(){
      //Slowly do angleClawUp
    m_clawTilterMotor.set(ArmTiltConstants.kClawSecondTierSpeed);
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
