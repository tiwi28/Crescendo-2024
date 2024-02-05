package frc.robot.commands;

import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command {
    private DoubleSupplier speedDoubleSupplier;
    private Shooter shooter;

    public void shoot(DoubleSupplier m_speedDoubleSupplier, Shooter shooter){
        this.shooter = shooter; 
        this.speedDoubleSupplier = m_speedDoubleSupplier;
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute(){
        var speed = speedDoubleSupplier.getAsDouble();
        shooter.shoot(speed);
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
