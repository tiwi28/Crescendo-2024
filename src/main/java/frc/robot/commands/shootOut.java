package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class shootOut extends Command {

    private ShooterConstants shooter;

    public void shoot(ShooterConstants shooter){
        this.shooter = shooter; 
    }

    @Override
    public void initialize(){
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
