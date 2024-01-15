package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax leader = new CANSparkMax(, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(2, MotorType.kBrushless);
  private final double[] intakePIDnum = Constants.IntakeConstants.intakeConstantsPID;

  public IntakeSubsystem() {
    leader.restoreFactoryDefaults();
    leader.setSmartCurrentLimit(40);
    leader.setIdleMode(IdleMode.kCoast);
    leader.enableVoltageCompensation(12);

    follower.restoreFactoryDefaults();
    follower.setSmartCurrentLimit(40);
    follower.setIdleMode(IdleMode.kCoast);
    follower.enableVoltageCompensation(12);
    follower.follow(leader, true);

  }

  private double[] intakePIDnum = Constants.IntakeConstants.intakeConstantsPID;
  private PIDController intakePID = new PIDController(intakePIDnum[0], intakePIDnum[1], intakePIDnum[2]);
  private double setpoint;
//   public Command intakeMethodCommand() {}

  /**
   * An intake method querying a boolean state of the subsystem (for intake, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean intakeCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void intakeIntake() {
    leader.set(intakePID.calculate(Constants.IntakeConstants.inEncoder.getDistance, setpoint));
  }

  @Override
  public void stop() {
    leader.set(0)
    }
}
