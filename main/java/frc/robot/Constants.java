// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.PIDConstants;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static enum ScoringLocation{
    SPEAKER,
    AMP,
    TRAP,
  }


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class IntakeConstants {
    public static final int intakeLeaderID = 1;
    public static final int intakeFollowerID = 3;
    public static final Encoder inEncoder = new Encoder(null, null);
    public static final double[] intakeConstantsPID = {0.5, 0, 0, Units.degrees};
  }
  public static class ShooterConstants {
    public static final int shooterMotorID = 2;
    public static final Encoder shootEncoder = new Encoder(null, null);
    public static final double[] shooterConstantsPID = {1, 0, 0, Units.degreesToRadians(0)};

    public void stop() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'stop'");
    } 

  } 
}
