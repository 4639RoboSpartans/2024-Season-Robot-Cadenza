package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.network.LimeLight;
import math.Averager;

import static frc.robot.Constants.RobotInfo.*;

@SuppressWarnings("unused")
public class FalconShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
    private final TalonFX shooterMotor;

    // TODO: extract 10 to a constant
    private final Averager shooterOutput = new Averager(2);
    private final BangBangController bangBangController;

    private boolean isShooterRunning = false;

    public FalconShooterSubsystem(int shooterMotorID) {
        shooterMotor = new TalonFX(shooterMotorID);

        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor.setInverted(true);

        shooterMotor.getConfigurator().apply(
            new CurrentLimitsConfigs().withStatorCurrentLimit(3)
        );

        bangBangController = new BangBangController();
    }

    @Override
    public void periodic() {
        if(!isShooterRunning) {
            shooterMotor.stopMotor();
        }
        else {
            ShooterInfo.ShooterSetpoint setpoint = ShooterMeasurementLERPer.get(
                LimeLight.getXDistance(),
                LimeLight.getZDistance()
            );

            double currentSpeed = getCurrentSpeed();
            double targetSpeed = setpoint.speed();

            double controllerOutput = bangBangController.calculate(currentSpeed, targetSpeed);

            shooterOutput.addMeasurement(controllerOutput);

            shooterMotor.set(shooterOutput.getValue() * ShooterInfo.MAX_SHOOTER_SPEED);
        }
    }

    private double getCurrentSpeed() {
        return shooterMotor.getVelocity().getValue();
    }

    @Override
    public boolean isUpToSpeed() {
        return Math.abs(getCurrentSpeed()) >= Math.abs(ShooterInfo.TARGET_SHOOTER_SPEED) * 0.95;
    }

    public void runShooter() {
        isShooterRunning = true;
    }

    public void stopShooter() {
        isShooterRunning = false;
    }
}