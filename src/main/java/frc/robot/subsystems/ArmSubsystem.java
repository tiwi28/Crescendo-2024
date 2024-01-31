// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax ArmFollower = new CANSparkMax(Constants.Arm.followerMotorID, MotorType.kBrushless);
  private final CANSparkMax ArmLeader = new CANSparkMax(Constants.Arm.leaderMotorID, MotorType.kBrushless);
  public ArmSubsystem() {
    ArmLeader.restoreFactoryDefaults();
    ArmLeader.setSmartCurrentLimit(40);
    ArmLeader.setIdleMode(IdleMode.kBrake);
    ArmLeader.enableVoltageCompensation(12);
    
    ArmFollower.restoreFactoryDefaults();
    ArmFollower.setSmartCurrentLimit(40);
    ArmFollower.setIdleMode(IdleMode.kBrake);
    ArmFollower.enableVoltageCompensation(12);
    ArmFollower.follow(ArmLeader);
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
