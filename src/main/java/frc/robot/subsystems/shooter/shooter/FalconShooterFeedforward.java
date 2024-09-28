package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.Helpers;

public class FalconShooterFeedforward extends ShooterSubsystem {
    private final TalonFX shooterLeft, shooterRight;
    private double targetVelocity;

    public FalconShooterFeedforward() {
        shooterLeft = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_LEFT_MOTOR, "Canivore1");
        shooterRight = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_RIGHT_MOTOR, "Canivore1");
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ShooterConstants.SHOOTER_PID_kp)
                                .withKI(ShooterConstants.SHOOTER_PID_ki)
                                .withKD(ShooterConstants.SHOOTER_PID_kd)
                                .withKV(ShooterConstants.SHOOTER_PID_kv))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(4)
                                .withStatorCurrentLimitEnable(true));
        shooterLeft.getConfigurator().apply(shooterConfig);
        shooterRight.getConfigurator().apply(shooterConfig);
        shooterLeft.setNeutralMode(NeutralModeValue.Coast);
        shooterRight.setNeutralMode(NeutralModeValue.Coast);
        shooterRight.setControl(new Follower(shooterLeft.getDeviceID(), true));
        targetVelocity = ShooterConstants.SHOOTER_IDLE.speed();
    }

    @Override
    protected boolean atSetPointSupplier() {
        return Helpers.withinTolerance(
                getCurrentSpeed(),
                targetVelocity,
                ShooterConstants.SHOOTER_SPEED_TOLERANCE
        );
    }

    @Override
    protected void runShootingSpeed(double speed) {
        targetVelocity = speed;
        shooterLeft.setControl(new VelocityDutyCycle(speed));
    }

    @Override
    protected double getCurrentSpeed() {
        return shooterLeft.getVelocity().getValueAsDouble();
    }
}