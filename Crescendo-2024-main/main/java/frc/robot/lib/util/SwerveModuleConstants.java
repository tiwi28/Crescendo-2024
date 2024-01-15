package frc.robot.lib.util;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final String cancoderCANBUS;
  public final double angleOffset;
  public final PIDConstants anglePID;
  public final PIDConstants drivePID;
  public final SVAConstants driveSVA;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID   drive controller ID
   * @param angleMotorID   angle controller ID
   * @param cancoderID     canCoder ID
   * @param cancoderCANBUS Can bus that the cancoder is on
   * @param angleOffset    canCoder offset
   * @param anglePID       angle motor PID values
   * @param anglePID       drive motor PID values
   * @param driveSVA       drive motor SVA values (feed forward)
   */

  public SwerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, String cancoderCANBUS,
      double angleOffset, PIDConstants anglePID, PIDConstants drivePID, SVAConstants driveSVA) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = cancoderID;
    this.cancoderCANBUS = cancoderCANBUS;
    this.angleOffset = angleOffset;
    this.anglePID = anglePID;
    this.drivePID = drivePID;
    this.driveSVA = driveSVA;
  }

}
