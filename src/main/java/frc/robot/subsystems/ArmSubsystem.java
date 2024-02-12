package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;

public class ArmSubsystem extends ProfiledPIDSubsystem {

    private final static DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);
    private final CANSparkMax leader = new CANSparkMax(Constants.Arm.leaderMotorID, MotorType.kBrushless);
    private final CANSparkMax follower = new CANSparkMax(Constants.Arm.followerMotorID, MotorType.kBrushless);

    private final static double offset = 1.7486;
    // in radians
    private final ArmFeedforward feedforward = new ArmFeedforward(0.05, 0.035, 0.052);
    public double target = 0;

    public ArmSubsystem() {
        super(
                new ProfiledPIDController(
                        0.05,
                        0.005,
                        0,
                        new TrapezoidProfile.Constraints(
                                Units.degreesToRadians(120), Units.degreesToRadians(300))));
              
        this.m_controller.setIntegratorRange(-0.001, 0.001);
        this.enable();
        this.setGoal(this.getMeasurement());
        this.leader.setSmartCurrentLimit(60, 40);
        follower.setSmartCurrentLimit(60, 40);
        leader.setIdleMode(IdleMode.kBrake);
        follower.setIdleMode(IdleMode.kBrake);

        follower.follow(leader);

    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double ffoutput = feedforward.calculate(setpoint.position, setpoint.velocity);
        SmartDashboard.putNumber("Arm velocity", setpoint.velocity);
        SmartDashboard.putNumber("Arm target", setpoint.position);
        SmartDashboard.putNumber("Arm output", output);

        leader.set(output + ffoutput);
    }

    @Override
    public double getMeasurement() {
        return Units.degreesToRadians(absoluteEncoder.getDistance() * (360 / 4)) - offset;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Arm rotation encoder radians",
                Units.degreesToRadians(absoluteEncoder.getDistance() * (360 / 4)) - offset);

        target = Units.degreesToRadians(absoluteEncoder.getDistance() * (360 / 4)) - offset;

        SmartDashboard.putNumber("Arm rotation encoder degrees",
                absoluteEncoder.getDistance() * (360 / 4) - Units.radiansToDegrees(offset));

    }

}