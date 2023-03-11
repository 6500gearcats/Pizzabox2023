package frc.robot.commands;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LightPurple extends CommandBase{
    
    private Solenoid PurpleLight = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

    @Override
    public void initialize() {
        PurpleLight.set(true);
    }

    public void end(boolean done) {
        PurpleLight.set(false);
    }

}
