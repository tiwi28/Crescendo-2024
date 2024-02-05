package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;

public class Intake extends SubsystemBase {
  private final CANSparkMax leader = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(2, MotorType.kBrushless);
  // private final double[] intakePIDnum = IntakeConstants.intakeConstantsPID;

  public Intake() {
    leader.restoreFactoryDefaults();
    leader.setSmartCurrentLimit(40);
    leader.setIdleMode(IdleMode.kCoast);
    leader.enableVoltageCompensation(12);

    follower.restoreFactoryDefaults();
    follower.setSmartCurrentLimit(40);
    follower.setIdleMode(IdleMode.kCoast);
    follower.enableVoltageCompensation(12);
    follower.follow(leader);


  }

  private PIDController intakePID = IntakeConstants.intakeConstantsPID.getController();
  private SimpleMotorFeedforward intakeFF = IntakeConstants.intakeConstantsFF.getController();
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


  public void intakeIntake() {
    leader.set(intakePID.calculate(IntakeConstants.inEncoder.getDistance(), setpoint) + intakeFF.calculate(0,0));
  }

  public void stop() {
    leader.set(0);
    
    }
}
