package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DriveAt4 extends Command {
    private final Swerve swerve;

    public DriveAt4(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        this.swerve.drive(new Translation2d(4, 0), 0, true, false, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerve.drive(new Translation2d(0, 0), 0, true, false, false, true);
    }
}
