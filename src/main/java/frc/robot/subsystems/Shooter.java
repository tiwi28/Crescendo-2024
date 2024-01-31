// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.util.PIDConstants;

public class Shooter extends SubsystemBase{
    // init of motors and pidcontrollers
    private CANSparkMax leader = new CANSparkMax(Constants.ShooterConstants.shooterLeaderID, MotorType.kBrushless);
    private CANSparkMax follower = new CANSparkMax(Constants.ShooterConstants.shooterFollowerID, MotorType.kBrushless);
    private CANSparkMax feeder = new CANSparkMax(Constants.ShooterConstants.feederID, MotorType.kBrushless);
    
    private PIDController shooterPID = ShooterConstants.shooterConstantsPID.getController();
    private PIDController feederPID = ShooterConstants.feederConstantsPID.getController();
    private SimpleMotorFeedforward shooterFF = ShooterConstants.shooterConstantsFF.getController();
    // private double setpoint;
    
    public Shooter() {
    
    leader.setSmartCurrentLimit(40);
    leader.setIdleMode(IdleMode.kCoast);
    leader.enableVoltageCompensation(12);

    follower.restoreFactoryDefaults();
    follower.setSmartCurrentLimit(40);
    follower.setIdleMode(IdleMode.kCoast);
    follower.enableVoltageCompensation(12);

    
    }
    public void shoot(double setpoint) {
        leader.setVoltage(shooterPID.calculate(Constants.ShooterConstants.shootEncoder.getDistance(), setpoint) + shooterFF.calculate(0, 0, 0));
    }

    public void feed(double setpoint) {
        feeder.set(feederPID.calculate(Constants.ShooterConstants.shootEncoder.getDistance(), setpoint));
    }

    public void stop() {
        leader.set(0);
    }

}