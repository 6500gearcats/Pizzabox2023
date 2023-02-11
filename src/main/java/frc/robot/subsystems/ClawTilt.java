package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ClawTiltConstants;

public class ClawTilt {

        // Create the tilter motor
        // The constants are not corect right now, will be replaced.
    
        private final MotorController m_clawTilterMotor = new CANSparkMax(ClawTiltConstants.kClawTiltMotorPort, MotorType.kBrushed);
    
        public ClawTilt() {}

        // NOTICE! The two blocks of code raise exceptions. Don't know why.

        /*@Override
        public void periodic() {
          // This method will be called once per scheduler run
        }
      
        @Override
        public void simulationPeriodic() {
          // This method will be called once per scheduler run during simulation
        }*/
            
        public void setReverse() {
          // Quickly reverse the tilt of the arm after depositing object.
          m_clawTilterMotor.set(ClawTiltConstants.kTiltReverseSpeed);
        }
    
        public void setForwardSpeed() {
            // Tilt arm quickly.
          m_clawTilterMotor.set(ClawTiltConstants.kTiltForwardSpeed);
        }
    
        // NOTICE! The following three methods may not be needed.
    
        public void setStowSpeed() {
            //Slowly reverse the tilt of the arm after arm nears robot.
          m_clawTilterMotor.set(ClawTiltConstants.kTiltStowSpeed);
        }
    
        public void setSecondTierSpeed() {
            // Tilt the arm slowly to deposit objects on the second teir.
          m_clawTilterMotor.set(ClawTiltConstants.kSecondTierSpeed);
        }
    
        public void stop() {
            // Stop the motor moving.
            m_clawTilterMotor.stopMotor();
    
    }
    
    
}
