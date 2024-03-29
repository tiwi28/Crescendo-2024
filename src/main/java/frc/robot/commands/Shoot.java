package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command {
  private Timer m_timer = new Timer();
  private final DoubleSupplier m_RPMSupplier;
  private final Shooter m_shooter;
  private TrapezoidProfile m_profiler = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          Constants.ShooterConstants.maxVelocityPerSecond,
          Constants.ShooterConstants.maxAcceleration));

  public Shoot(DoubleSupplier targetRPM, Shooter shooter) {
    m_RPMSupplier = targetRPM;
    m_shooter = shooter;

    this.addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    var nextState = m_profiler.calculate(m_timer.get(),
        new TrapezoidProfile.State(m_shooter.getActualRPM(), 0),
        new TrapezoidProfile.State(m_RPMSupplier.getAsDouble(), 0));

    m_shooter.runVelocity(nextState.position, nextState.velocity);
  }

  @Override
  public boolean isFinished() {
    return m_profiler.isFinished(m_timer.get()) && m_shooter.atSetpoint();
  }

}