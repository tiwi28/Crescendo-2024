package frc.robot.commands;

import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class Feed extends Command {

    public Shooter shooter;
    private DoubleSupplier speedDoubleSupplier;
   
      
    public void feed(DoubleSupplier m_speedDoubleSupplier, Shooter shooter){
        this.shooter = shooter;
        this.speedDoubleSupplier = m_speedDoubleSupplier; 
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        var speed = speedDoubleSupplier.getAsDouble();
        shooter.feed(speed);

    }

    @Override
    public void end(boolean interrupted) {
         
    }

    @Override
    public boolean isFinished() {
       shooter.stop();
    return true;
    }
}