package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.Helpers;
import frc.robot.subsystems.shooter.ShooterConstants;

import static edu.wpi.first.units.Units.*;

public class SimShooterSubsystem extends ShooterSubsystem {
    private final DCMotorSim simShooter;
    private final TalonFX shooter;
    private double targetVelocity;

    private final SysIdRoutine routine;

    public SimShooterSubsystem() {
        shooter = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_LEFT_MOTOR, "Canivore1");
        simShooter = new DCMotorSim(DCMotor.getFalcon500(2), 1.5, 0.1);
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withKP(ShooterConstants.SIM_SHOOTER_PID_kp)
                                .withKI(ShooterConstants.SIM_SHOOTER_PID_ki)
                                .withKD(ShooterConstants.SIM_SHOOTER_PID_kd)
                                .withKV(ShooterConstants.SIM_SHOOTER_PID_kv)
                                .withKA(ShooterConstants.SIM_SHOOTER_PID_ka))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(4)
                                .withStatorCurrentLimitEnable(true));
        shooter.getConfigurator().apply(shooterConfig);
        shooter.setNeutralMode(NeutralModeValue.Coast);
        targetVelocity = ShooterConstants.SHOOTER_IDLE.speed();

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> output) -> runShootingVoltage(output.in(Volts)),
                        log -> {
                            log.motor("shooter")
                                    .voltage(Volts.of(shooter.getSimState().getMotorVoltage() * RobotController.getBatteryVoltage()))
                                    .angularPosition(Radians.of(simShooter.getAngularPositionRad()))
                                    .angularVelocity(RadiansPerSecond.of(simShooter.getAngularVelocityRadPerSec()));
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
        shooter.setControl(new VelocityTorqueCurrentFOC(speed));
        var talonFXSim = shooter.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltage();
        simShooter.setInputVoltage(motorVoltage);
        simShooter.update(0.02);
        talonFXSim.setRawRotorPosition(simShooter.getAngularPositionRotations());
        talonFXSim.setRotorVelocity(getCurrentSpeed());
    }

    private void runShootingVoltage(double voltage) {
        shooter.setControl(new VoltageOut(voltage));
        var talonFXSim = shooter.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltage();
        simShooter.setInputVoltage(motorVoltage);
        simShooter.update(0.02);
        talonFXSim.setRawRotorPosition(simShooter.getAngularPositionRotations());
        talonFXSim.setRotorVelocity(getCurrentSpeed());
    }

    @Override
    protected double getCurrentSpeed() {
        return Rotation2d.fromRadians(simShooter.getAngularVelocityRadPerSec()).getRotations();
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");
        builder.addDoubleProperty("Speed",
                this::getCurrentSpeed,
                null);
        builder.addDoubleProperty("Target speed",
                () -> targetVelocity,
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
