package frc.robot.commands;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LightPurple extends CommandBase {
    
    private Solenoid purpleLight = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    
    public void initialize() {
        purpleLight.set(true);
    }

    public void end(boolean done) {
        purpleLight.set(false);
    }

}
