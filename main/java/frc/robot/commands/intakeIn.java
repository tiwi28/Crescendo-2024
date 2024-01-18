
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeIn {
/** An Intake command that uses an Intake subsystem. */
  public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem m_subsystem;

  /**
   * Creates a new IntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public void intake(Intake intake){
    this.intake = intake; 
    addRequirements(intake);
}

  // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      intake.intakeIntake()
    }

  // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      intake.stop()
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    }
}

}
