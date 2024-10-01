package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.lib.util.Helpers;

public class ConcretePivotSubsystem extends PivotSubsystem {
    private final CANSparkMax leftPivot, rightPivot;
    private final DutyCycleEncoder pivotEncoder;

    private final ProfiledPIDController pivotPID;

    private MechanismRoot2d pivotRoot;
    private MechanismLigament2d pivot;

    public ConcretePivotSubsystem() {
        leftPivot = new CANSparkMax(ShooterConstants.IDs.SHOOTER_PIVOT_MOTOR_LEFT, MotorType.kBrushless);
        rightPivot = new CANSparkMax(ShooterConstants.IDs.SHOOTER_PIVOT_MOTOR_RIGHT, MotorType.kBrushless);
        leftPivot.restoreFactoryDefaults();
        rightPivot.restoreFactoryDefaults();

        rightPivot.follow(leftPivot, true);
        leftPivot.setSmartCurrentLimit(40);
        rightPivot.setSmartCurrentLimit(40);

        pivotEncoder = new DutyCycleEncoder(ShooterConstants.IDs.SHOOTER_PIVOT_ENCODER_DIO_PORT);
        pivotEncoder.setDutyCycleRange(-2, 2);

        pivotPID = new ProfiledPIDController(
                ShooterConstants.SHOOTER_PIVOT_kp,
                ShooterConstants.SHOOTER_PID_ki,
                ShooterConstants.SHOOTER_PID_kd,
                new Constraints(
                        ShooterConstants.SHOOTER_PIVOT_VELOCITY,
                        ShooterConstants.SHOOTER_PIVOT_ACCELERATION
                )
        );
    }

    @Override
    public void periodic() {
        runShootingAngle(getTargetAngle());
        pivot.setAngle(measuredPivotRotationsToMechanismDegrees(getRotations()));
    }

    public double getRotations() {
        return pivotEncoder.getAbsolutePosition();
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        builder.addDoubleProperty("Rotations", this::getRotations, null);
        builder.addDoubleProperty("PID output",
                () -> {
                    return pivotPID.calculate(getRotations());
                },
                null);
        builder.addDoubleProperty("Left output", leftPivot::getAppliedOutput, null);
        builder.addDoubleProperty("Right output", rightPivot::getAppliedOutput, null);
    }

    @Override
    public void initMech(Mechanism2d mech) {
        pivotRoot = mech.getRoot("Pivot", 3, 1);
        pivot = pivotRoot.append(
                new MechanismLigament2d("Shooter", 1, getRotations())
        );
    }

    @Override
    public void runShootingAngle(double angle) {
        pivotPID.setGoal(angle);
        double PIDOutput = pivotPID.calculate(getRotations());
        leftPivot.set(PIDOutput);
    }

    @Override
    protected boolean atSetPointSupplier() {
        return Helpers.withinTolerance(
                getRotations(),
                pivotPID.getGoal().position,
                ShooterConstants.SHOOTER_PIVOT_TOLERANCE
        );
    }

    private double measuredPivotRotationsToMechanismDegrees(double rotations) {
        double angle = Rotation2d.fromRotations(rotations).getDegrees() % 360;
        return angle + ShooterConstants.SHOOTER_PIVOT_MECHANISM_OFFSET_DEGREES;
    }
}
