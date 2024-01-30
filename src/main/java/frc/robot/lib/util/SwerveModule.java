package frc.robot.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  /* Module details */
  public int moduleNumber;

  /* Motors */
  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  /* Encoders and their values */
  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;
  private double lastAngle;
  private double angleOffset;

  /* Controllers */
  public final SparkPIDController driveController;
  public final SparkPIDController angleController;
  public final PIDConstants anglePID;
  public final SVAConstants driveSVA;
  public SimpleMotorFeedforward feedforward;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;

    angleOffset = moduleConstants.angleOffset;
    this.anglePID = moduleConstants.anglePID;
    this.driveSVA = moduleConstants.driveSVA;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, moduleConstants.cancoderCANBUS);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();

    // ANGLE PID DASHBOARD
    // this.anglePID.sendDashboard(Constants.Swerve.moduleNames[this.moduleNumber] +
    // "Angle");
    // DRIVE PID DASHBOARD
    // this.drivePID.sendDashboard(Constants.Swerve.moduleNames[this.moduleNumber] +
    // "Drive");
    // DRIVE SVA DASHBOARD
    // this.driveSVA.sendDashboard(Constants.Swerve.moduleNames[this.moduleNumber] +
    // "Drive");

    // this.updateDashboardValues();
  }

  public void updateDashboardValues() {
    SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Integrated Encoder",
        this.getState().angle.getDegrees());
    SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Can Coder",
        this.getCanCoder().getDegrees());
    SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Set Point", this.lastAngle);
    SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + " Drive Encoder Velocity",
        this.driveEncoder.getVelocity());
  }

  public void updateControllerValues() {
    this.feedforward = this.driveSVA.retrieveDashboard();
    this.anglePID.retrieveDashboard(this.angleController);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // this.updateControllerValues();
    desiredState = OnboardModuleState.optimize(
        desiredState,
        getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which REV and CTRE are not

    this.setSpeed(desiredState, isOpenLoop);
    this.setAngle(desiredState);
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    angleEncoder.getConfigurator().apply(config);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    this.anglePID.applyPID(this.angleController);
    angleController.setFF(0);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    Constants.Swerve.drivePID.applyPID(this.driveController);
    driveController.setFF(0);
    this.feedforward = driveSVA.getController();
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveEncoder.setPosition(0.0);

    if (this.moduleNumber == 1 || this.moduleNumber == 3) {
      driveMotor.setInverted(!Constants.Swerve.driveInvert);
    } else {
      driveMotor.setInverted(Constants.Swerve.driveInvert);

    }
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
      // SmartDashboard.putNumber(Constants.Swerve.moduleNames[this.moduleNumber] + "
      // Drive Set Velocity",
      // desiredState.speedMetersPerSecond);
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle
            .getDegrees();

    angleController.setReference(angle, ControlType.kPosition);
    lastAngle = angle;
  }

  public void goToHome() {
    Rotation2d angle = getAngle();
    angleController.setReference(angle.getDegrees() - angle.getDegrees() % 360,
        ControlType.kPosition);
    lastAngle = angle.getDegrees() - angle.getDegrees() % 360;
  }

  public Rotation2d getCanCoder() {
    double angle = Math.toRadians(360.0 * angleEncoder.getAbsolutePosition().getValueAsDouble() - this.angleOffset);
    if (angle < 0) {
      angle = Math.PI * 2 + angle;
    }
    return Rotation2d.fromRadians(angle);
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(this.integratedAngleEncoder.getPosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getSpeed(), this.getAngle());
  }

  public double getSpeed() {
    return this.driveEncoder.getVelocity();
  }

  public double getDistance() {
    return this.driveEncoder.getPosition();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(this.getDistance(), this.getAngle());
  }

  public SwerveModulePosition getRedPosition() {
    return new SwerveModulePosition(this.getDistance(), Rotation2d.fromDegrees(-this.getAngle().getDegrees()));
  }

  public CANSparkMax getDriveMotor() {
    return this.driveMotor;
  }
}