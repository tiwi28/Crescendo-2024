package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmAiming extends Command{
    
   public ArmSubsystem arm;
   private double angle;
   

   public void aim(ArmSubsystem arm){
    this.arm = arm;
    this.angle = angle;
    
   }

   @Override
   public void initialize(){

   }

   @Override
   public void execute(){

   }

   @Override
   public void end(boolean interrupted){
    
   }
   
}
