package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.lib.util.SVAConstants;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.lib.util.PIDConstants;
import frc.robot.lib.util.ProfiledPIDConstants;

public final class Constants {
  // public static enum PieceType {
  //   CONE,
  //   CUBE,
  //   AIR
  // }

  // public static enum GamePieceLevel {
  //   L1,
  //   L2,
  //   L3,
  // } 
  // Unneeded until further notice

  public static enum RobotModes {
    Competition,
    Debug
  }

  public static final RobotModes robotMode = RobotModes.Debug;

  public static final class Vision {
    public static final String rightCameraName = "OV5647";
    public static final String leftCameraName = "OV5647left";
    public static final Transform3d cameraToRobot = new Transform3d(
        new Translation3d(-0.22, 0.35, 0),
        new Rotation3d(0, Units.degreesToRadians(0), 0));
    public static final Transform3d leftCameraToRobot = new Transform3d(
        new Translation3d(-0.22, -0.35, 0),
        new Rotation3d(0, Units.degreesToRadians(0), 0));

    public static final Transform3d robotToCamera = cameraToRobot.inverse();
    public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    public static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 2);
    public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);

    /* Custom PID Controllers for Vision */
    public static final ProfiledPIDController translationController = new ProfiledPIDController(3, 0, 0.2,
        TRANSLATION_CONSTRAINTS);
    public static final ProfiledPIDController rotationController = new ProfiledPIDController(3, 0, 0.2,
        ROTATION_CONSTRAINTS);
  }

  public static final class Operators {
    public static final int driver = 0;
    public static final int operator = 1;
  }

  public static final class FieldConstants {

    public static double fieldLength;
    public static double fieldWidth;

  }

  public static final class Arm {
    public static final int leaderMotorID = 31;
    public static final int followerMotorID = 32;
   
     public static final int armEncoderDIOPort = 0;
    public static final double encoderOffset = Units.degreesToRadians(83.6 + 6.2 + 6.6 + 5.3); // En`coder reading when
                                                                                               // the
    // arm is
    // parallel
    // to the ground.
    public static final ArmFeedforward armFF = new ArmFeedforward(0, 0.92, 1.09, 0.04); // until we have some empirical
    // measurement, kS will be added
    public static final PIDConstants armPID = new PIDConstants(1, 0, 0.05, 0);
    public static final double armMaxOutput = 0.8;
    public static final double kMaxVelocityRadPerSecond = 0.3;
    public static final double kMaxAccelerationRadPerSecSquared = 0.1;

    public static final int currentLimit = 40;
    public static final int stallLimit = 80;
    public static final double gearRatio = 56; // 56:1

    public static final double maxAngle = Units.degreesToRadians(60);
    public static final double minAngle = Units.degreesToRadians(-58.3);

    public static final double commandTimeout = 1.5;
  }

  public static final class IntakeConstants {
    //PIDFF
    public static final PIDConstants intakeConstantsPID = new PIDConstants(0, 0, 0);
    public static final SVAConstants intakeConstantsFF = new SVAConstants(0, 0, 0);
    
    // ID's + Encoder
    public static final int intakeLeaderID = 1;
    public static final int intakeFollowerID = 3;
    public static final Encoder inEncoder = new Encoder(null, null);
  }
  
  public static final class ShooterConstants {
    //ID's
    public static final int shooterLeaderID = 0;
    public static final int shooterFollowerID = 0;
    public static final int feederID = 0;
    public static final Encoder shootEncoder = new Encoder(null, null);
    //PID
    public static final ProfiledPIDConstants shooterConstantsPID = new ProfiledPIDConstants(0, 0, 0, 0, 0);
    public static final PIDConstants feederConstantsPID = new PIDConstants(0,0,0);
    public static final SVAConstants shooterConstantsFF = new SVAConstants(0, 0, 0);

    public static final int id = 5;
    public static final double[] shooterPID = { 0, 0, 0 };
    public static final double[] shooterFeedforward = { 0, 0 };
    public static final int toleranceRPM = 10;
    public static final double maxVelocityPerSecond = 2000; // RPM/s
    public static final double maxAcceleration = 700; // RPM/s^2

  }

  // public static final class Wrist {
  //   public static final int intakeMotorID = 9;
  //   public static final int wristMotorID = 33;

  //   public static final PIDConstants wristRotationPID = new PIDConstants(0.7, 0.6, 0.0005, Units.degreesToRadians(10));
  //   // public static final PIDConstants wristRotationPID = new PIDConstants(0.7,
  //   // 0.6, 0.0005, Units.degreesToRadians(10));
  //   public static final double wristGearRatio = 67.2;

  //   public static final double intakePower = 0.5;

  //   public static final double intakeInCube = 0.5;
  //   public static final double intakeInCone = -0.5;

  //   public static final double outakeConeL1 = 0.25;
  //   public static final double outakeConeL2 = 0.51735;
  //   public static final double outakeConeL3 = 1;

  //   public static final double outakeCubeL2 = -0.5;
  //   public static final double outakeCubeL3 = -0.6;

  //   public static final int absoluteEncoderPort = 1;
  //   public static final double positionOffset = 4.96 + 0.035;

  //   public static final double maxAngle = 3.18;
  //   public static final double minAngle = -1.664522;

  //   public static final int beambreakDIO = 2;

  //   public static final int CubeHPDelay = 600; // for cube HP delay in miliseconds

  //   public static final double commandTimeout = 1.5;

  //   public static final PIDConstants intakePIDConstants = new PIDConstants(0.1, 0, 0);
  // }

  public static final class Swerve {
    /* Drive Controls */
    public static final double stickDeadband = 0.1;

    /* Auto Constants */
    public static HolonomicPathFollowerConfig holonomicConfig;
    /* Gyro */
    public static final int pigeonID = 21;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    public static final String pigeonCanBUS = "rio";

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23.0);
    public static final double wheelBase = Units.inchesToMeters(23.0);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final Matrix<N3, N1> stateStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(),
        new double[] { 0.05, 0.05, Units.degreesToRadians(2) });

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = 150.0 / 7.0; // 150/7:1

    /* Custom PID Controllers */
    public static final PIDConstants robotRotationPID = new PIDConstants(0.1, 0, 0.00005);
    public static final PIDConstants translationPID = new PIDConstants(4, 0, 0.005);
    public static final PIDConstants balancePID = new PIDConstants(0.043, 0.0, 0, 2);

    /* Delays (milliseconds) */
    public static final long defenseDelay = 500;

    /* Kinematics */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 40;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 6; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    public static final String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" }; // module #0,
    // #1, #2, #3

    public static final SVAConstants driveSVA = new SVAConstants(0.05, 2.5, 0.5);
    public static final PIDConstants drivePID = new PIDConstants(0.001, 0.00005, 0.0005);

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 14;
      public static final int angleMotorID = 13;
      public static final int canCoderID = 22;
      public static final String cancoderCANBUS = "rio"; // change to "rio" if it's on rio
      public static final double angleOffset = 23.1;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 16;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 24;
      public static final String cancoderCANBUS = "rio";
      public static final double angleOffset = 141.5;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 23;
      public static final String cancoderCANBUS = "rio";
      public static final double angleOffset = 56.1;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 18;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 25;
      public static final String cancoderCANBUS = "rio";
      public static final double angleOffset = 271.75;
      public static final PIDConstants anglePID = new PIDConstants(0.02, 0.0, 0.005);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, cancoderCANBUS, angleOffset, anglePID, drivePID, driveSVA);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // public static final PIDConstants translationPID = new PIDConstants(0.8, 0.2,
    // 0.05);
    public static final PIDConstants translationPID = new PIDConstants(3, 0.5, 0);
    public static final PIDConstants rotationPID = new PIDConstants(2, 0.002, 0.05);

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class LEDConstants {
    public static final int blinkinPort = 0;
    public static final int blinkPerSecond = 50 / 5;
    public static final double forestPattern = -0.37;
    public static final double colorGradient = 0.41;
    public static final double solidYellow = 0.69;
    public static final double solidViolet = 0.91;
    public static final double solidGreen = 0.77;
    public static final double solidRed = 0.61;
    public static final double solidBlue = 0.87;
    public static final double solidOrange = 0.65;
    public static final double skyblue = 0.83;
    public static final double off = 0.99;
    public static final double rainbow = -0.99;
    public static final double hotPink = 0.57;
    public static final double darkRed = 0.59;
    public static final double red = 0.61;
    public static final double redOrange = 0.63;
    public static final double orange = 0.65;
    public static final double gold = 0.67;
    public static final double yellow = 0.69;
    public static final double lawnGreen = 0.71;
    public static final double lime = 0.73;
    public static final double darkGreen = 0.75;
    public static final double green = 0.77;
  }

 
}
