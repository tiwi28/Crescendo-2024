// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.lib.logging.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.util.PIDConstants;

public class Shooter extends SubsystemBase{


    /* init of motors and pidcontrollers */
    private CANSparkMax leader = new CANSparkMax(Constants.ShooterConstants.shooterLeaderID, MotorType.kBrushless);
    private CANSparkMax follower = new CANSparkMax(Constants.ShooterConstants.shooterFollowerID, MotorType.kBrushless);
    private final RelativeEncoder shootEncoder = leader.getEncoder();
    private CANSparkMax feeder = new CANSparkMax(Constants.ShooterConstants.feederID, MotorType.kBrushless);
    
    /*PIDFF and stuff I've never done before */
    // private PIDController shooterPID = ShooterConstants.shooterConstantsPID.getController();
    private ProfiledPIDController shooterPID = ShooterConstants.shooterConstantsPID.getController();
    private SimpleMotorFeedforward shooterFF = ShooterConstants.shooterConstantsFF.getController();
    // private PIDController feederPID = ShooterConstants.feederConstantsPID.getController();

    // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    // private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double velocitySetpoint = 0;
    private double velocityRateOfChange = 0;

    private LoggedTunableNumber shooterKp = new LoggedTunableNumber("shooterKp",
    Constants.ShooterConstants.shooterPID[0]);
private LoggedTunableNumber shooterKi = new LoggedTunableNumber("shooterKi",
    Constants.ShooterConstants.shooterPID[1]);
private LoggedTunableNumber shooterKd = new LoggedTunableNumber("shooterKd",
    Constants.ShooterConstants.shooterPID[2]);
private LoggedTunableNumber shooterKs = new LoggedTunableNumber("shooterKs",
    Constants.ShooterConstants.shooterFeedforward[0]);
private LoggedTunableNumber shooterKv = new LoggedTunableNumber("shooterKv",
    Constants.ShooterConstants.shooterFeedforward[1]);
    
    public Shooter() {
         
    leader.restoreFactoryDefaults();
    leader.setSmartCurrentLimit(40);
    leader.setIdleMode(IdleMode.kCoast);
    leader.enableVoltageCompensation(12);

    follower.restoreFactoryDefaults();
    follower.setSmartCurrentLimit(40);
    follower.setIdleMode(IdleMode.kCoast);
    follower.enableVoltageCompensation(12);
    follower.follow(leader);

    setupMotor();
     shootEncoder.setVelocityConversionFactor(1);
    
    //profile.calculate(5, m_setpoint, m_goal);
    
    }
    public void shoot(double setpoint) {
        leader.set(shooterPID.calculate(Constants.ShooterConstants.shootEncoder.getDistance(), setpoint) + shooterFF.calculate(setpoint, Constants.ShooterConstants.maxAcceleration));
    }

    public void feed(double speed) {
        feeder.set(speed);
    }

  public void setupMotor() {

    shooterPID.setP(Constants.ShooterConstants.shooterPID[0]);
    shooterPID.setI(Constants.ShooterConstants.shooterPID[1]);
    shooterPID.setD(Constants.ShooterConstants.shooterPID[2]);
  }

  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    if (shooterKp.hasChanged() || shooterKi.hasChanged() || shooterKd.hasChanged()) {
      shooterPID.setP(shooterKp.get());
      shooterPID.setI(shooterKi.get());
      shooterPID.setD(shooterKd.get());
    }
    if (shooterKs.hasChanged() || shooterKv.hasChanged()) {
      shooterFF = new SimpleMotorFeedforward(shooterKs.get(), shooterKv.get());
    }
  }

  public void runVelocity(double rpm, double rpmPerSecond) {
    this.velocitySetpoint = rpm;
    this.velocityRateOfChange = rpmPerSecond;
  }

  public void stop() {
   leader.set(0);
  }

  public double getActualRPM() {
    return shootEncoder.getVelocity();
  }

  public double getDesiredRPM() {
    return velocitySetpoint;
  }

  public double getDesiredRPMPerSecond() {
    return velocityRateOfChange;
  }

  public boolean atSetpoint() {
    return Math.abs(getActualRPM() - velocitySetpoint) < Constants.ShooterConstants.toleranceRPM;
  }

  public void logValues() {
    SmartDashboard.putNumber("Shooter Actual RPM", getActualRPM());
    SmartDashboard.putNumber("Shooter Desired RPM", getDesiredRPM());
    SmartDashboard.putNumber("Shooter Desired RPM/s", velocityRateOfChange);
  }

  @Override
  public void periodic() {
    double feedforward = shooterFF.calculate(velocitySetpoint, velocityRateOfChange);
    shooterPID.setReference(velocitySetpoint, ControlType.kVelocity, 0, feedforward);

    checkTunableValues();
    logValues();
  }
}