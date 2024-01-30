// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.util.PIDConstants;

public class Shooter extends SubsystemBase{
    // private double[] shooterPIDnum = Constants.ShooterConstants.shooterConstantsPID;
    private CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
    private PIDController shooterPID = ShooterConstants.shooterConstantsPID.getController();
    private double setpoint;
    

    public void shoot() {
        shooterMotor.set(shooterPID.calculate(Constants.ShooterConstants.shootEncoder.getDistance(), setpoint));
    }

    public void feed() {
        shooterMotor.set(-shooterPID.calculate(Constants.ShooterConstants.shootEncoder.getDistance(), setpoint));
    }

    public void stop() {
        shooterMotor.set(0);
    }

}