package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class shootOut extends Command {

    public Shooter shooter;

    public void shoot(Shooter shooter){
        this.shooter = shooter; 
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute(){
        shooter.out();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop(); 
    }

    @Override
    public boolean isFinished() {
       shooter.stop();
    return true;
    }

}
