package frc.robot.commands;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LightYellow extends CommandBase {
    
    private Solenoid yellowLight = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    
    public void initialize() {
        yellowLight.set(true);
    }

    public void end(boolean done) {
        yellowLight.set(false);
    }

}
