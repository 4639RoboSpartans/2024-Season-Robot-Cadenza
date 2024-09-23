package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.InterpolatingTables;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.util.AimUtil;
import frc.robot.util.Helpers;

public class Pivot extends SubsystemBase implements IPivot {
    private final CANSparkMax pivotLeft, pivotRight;
    private final DutyCycleEncoder encoder;
    private final ProfiledPIDController pivotPID;
    private final Trigger atSetPointTrigger;

    private static Pivot instance;

    private Pivot() {
        pivotLeft = new CANSparkMax(ShooterConstants.IDs.SHOOTER_PIVOT_MOTOR_LEFT,
                CANSparkMax.MotorType.kBrushless);
        pivotRight = new CANSparkMax(ShooterConstants.IDs.SHOOTER_PIVOT_MOTOR_RIGHT,
                CANSparkMax.MotorType.kBrushless);
        pivotLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotRight.follow(pivotLeft);

        encoder = new DutyCycleEncoder(ShooterConstants.IDs.SHOOTER_PIVOT_ENCODER_DIO_PORT);
        pivotPID = new ProfiledPIDController(
                4, 0, 0,
                new TrapezoidProfile.Constraints(
                        2.5, 1.5
                )
        );
        atSetPointTrigger = new Trigger(this::atSetPointSupplier);
    }

    private Command runPivot() {
        return run(() -> {
            double PIDOutput = pivotPID.calculate(encoder.getAbsolutePosition());
            pivotLeft.set(PIDOutput);
        });
    }

    @Override
    public Command autoShoot() {
        return runOnce(() -> pivotPID.setGoal(
                InterpolatingTables.getAngleTable().get(AimUtil.getSpeakerDist())
        )).andThen(runPivot());
    }

    @Override
    public Command manualShoot() {
        return runOnce(() -> pivotPID.setGoal(
                InterpolatingTables.getAngleTable().get(2.0)
        )).andThen(runPivot());
    }

    @Override
    public Command launch() {
        return runOnce(() -> pivotPID.setGoal(
                ShooterConstants.SHOOTER_LAUNCH_ANGLE
        )).andThen(runPivot());
    }

    @Override
    public Command idle() {
        return runOnce(() -> pivotPID.setGoal(
                ShooterConstants.SHOOTER_IDLE_SETPOINT
        )).andThen(runPivot());
    }

    @Override
    public Command stop() {
        return runOnce(() -> pivotPID.setGoal(
                encoder.getAbsolutePosition()
        )).andThen(runPivot());
    }

    private boolean atSetPointSupplier() {
        return Helpers.withinTolerance(
                encoder.getAbsolutePosition(),
                pivotPID.getGoal().position,
                ShooterConstants.SHOOTER_PIVOT_ERROR
        );
    }

    @Override
    public Trigger atSetPoint() {
        return atSetPointTrigger;
    }

    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }
}
