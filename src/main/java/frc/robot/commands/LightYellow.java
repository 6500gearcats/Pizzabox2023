package frc.robot.commands;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LightYellow extends CommandBase{
    
    private Solenoid YellowLight = new Solenoid(PneumaticsModuleType.CTREPCM, 6);

    @Override
    public void initialize() {
        YellowLight.set(true);
    }

    public void end(boolean done) {
        YellowLight.set(false);
    }

}
