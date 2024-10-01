package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.lib.util.Helpers;
import static edu.wpi.first.units.Units.*;

public class FalconShooterFeedforward extends ShooterSubsystem {
    private final TalonFX shooterLeft, shooterRight;
    private double targetVelocity;

    private final SysIdRoutine routine;

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

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> output) -> shooterLeft.setControl(new VoltageOut(output.in(Volts))),
                log -> {
                    log.motor("intake-pivot")
                            .voltage(Volts.of(shooterLeft.get() * RobotController.getBatteryVoltage()))
                            .angularPosition(Rotations.of(shooterLeft.getPosition().getValueAsDouble()))
                            .angularVelocity(RotationsPerSecond.of(shooterLeft.getVelocity().getValueAsDouble()));
                }, this)
        );
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

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");
        builder.addDoubleProperty("Speed",
                this::getCurrentSpeed,
                null);
    }

    @Override
    public Command getSysIDQuasistaticCommand() {
        return routine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    @Override
    public Command getSysIDDynamicCommand() {
        return routine.dynamic(SysIdRoutine.Direction.kForward);
    }
}