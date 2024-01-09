// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{

    private CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterConstants.shooterMotorID);
    private PIDController shooterPID = Constants.ShooterConstants.shooterConstantsPID;


    public void out() {
        shooterMotor.set(shooterPID);
    }

    public void in() {
        shooterMotor.set(shooterPID);
    }

    public void stop() {
        shooterMotor.set(0.0);
    }

}