package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmTiltConstants;

public class ArmTilt extends SubsystemBase {

    // Create the tilter motor
    // The constants are not corect right now, will be replaced.

    private final MotorController m_tilterMotor = new CANSparkMax(ArmTiltConstants.kTiltMotorPort, MotorType.kBrushed);

    public ArmTilt() {}

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
        
    public void setReverse() {
      // Quickly reverse the tilt of the arm after depositing object.
      m_tilterMotor.set(ArmTiltConstants.kTiltReverseSpeed);
    }

    public void setForwardSpeed() {
        // Tilt arm quickly.
      m_tilterMotor.set(ArmTiltConstants.kTiltForwardSpeed);
    }

    // NOTICE! The following three methods may not be needed.

    public void setStowSpeed() {
        //Slowly reverse the tilt of the arm after arm nears robot.
      m_tilterMotor.set(ArmTiltConstants.kTiltStowSpeed);
    }

    public void setSecondTierSpeed() {
        // Tilt the arm slowly to deposit objects on the second teir.
      m_tilterMotor.set(ArmTiltConstants.kSecondTierSpeed);
    }

    public void stop() {
        // Stop the motor moving.
        m_tilterMotor.stopMotor();
      }

}
