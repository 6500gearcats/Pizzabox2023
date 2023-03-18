package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase{
     public boolean slowClaw = false;
    
    private final MotorController m_clawTiltMotor = new CANSparkMax(ClawConstants.kClawMotorPort, MotorType.kBrushless);
    private final DoubleSolenoid m_clawMotor = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
    private final DutyCycleEncoder m_clawTiltEncoder = new DutyCycleEncoder(1);
    private final SlewRateLimiter clawFilter = new SlewRateLimiter(0.6);
    private double ClawPosition;

    public Claw() {
        //m_clawTiltEncoder.reset();
    }
    @Override
    public void periodic() {
        ClawPosition = m_clawTiltEncoder.getAbsolutePosition();


        SmartDashboard.putNumber("Claw Encoder:", ClawPosition);
        SmartDashboard.putString("clawSolonoid", m_clawMotor.get().toString());
    }

    public double getClawAngle() {
        return ClawPosition;
    }

    //Contol open and close positions of claw using double solenoid
    public void openClaw() {
        m_clawMotor.set(Value.kForward);
    }

    public void closeClaw() {
        m_clawMotor.set(Value.kReverse);
    }

    public void stopClaw() {
        m_clawMotor.set(Value.kOff);
    }



    //Control Tilt of the Claw, sets a speed not sure how to know when to stop, but we figure that out later
    public void clawUp() {
        m_clawTiltMotor.set(ClawConstants.kClawForwardSpeed);
    }
    
    public void clawUpSpeed(double speed) {
        if(slowClaw)
        {
          speed *= ClawConstants.kSlowClawModifier;
        }
        m_clawTiltMotor.set(clawFilter.calculate(speed));
      }

    public void clawDown() {
        m_clawTiltMotor.set(ClawConstants.kClawReverseSpeed);
    }

    public void clawDownSpeed(double speed) {
        if(slowClaw)
        {
          speed *= ClawConstants.kSlowClawModifier;
        }
        m_clawTiltMotor.set(clawFilter.calculate(speed));
      }
  

    public void stopClawTilt() {
        m_clawTiltMotor.stopMotor();
      }

    public void moveTo(double targetAngle) {
        
        if(getClawAngle() < targetAngle - ClawConstants.kClawTolerance){
            clawUp();
        }
        else if(getClawAngle() > targetAngle + ClawConstants.kClawTolerance){
            clawDown();
        }
        else{
            stopClawTilt();
        }
        
        
    }

}
