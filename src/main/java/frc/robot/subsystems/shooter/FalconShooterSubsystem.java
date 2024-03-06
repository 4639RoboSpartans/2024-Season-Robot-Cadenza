package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.pivot.IShooterPivotSubsystem;
import frc.robot.subsystems.aim.AimSubsystem;
import math.Averager;

import static frc.robot.Constants.RobotInfo.ShooterInfo;
import static frc.robot.Constants.RobotInfo.ShooterInfo.*;

public class FalconShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
    // Components
    private final TalonFX shooterMotor;
    private final IShooterPivotSubsystem shooterPivot;
    private final AimSubsystem aimSubsystem;
    // Controllers
    private final Averager shooterOutputAverager;
    private final BangBangController bangBangController;
    // State
    private ShootingMode shootingMode;

    public FalconShooterSubsystem(int shooterMotorID) {
        this.aimSubsystem = SubsystemManager.getAimSubsystem();
        this.shooterPivot = SubsystemManager.getShooterPivot(this);

        shooterMotor = new TalonFX(shooterMotorID);
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor.setInverted(true);
        shooterMotor.getConfigurator().apply(
            new CurrentLimitsConfigs().withStatorCurrentLimit(4)
        );

        shooterOutputAverager = new Averager(Constants.POSE_WINDOW_LENGTH);

        bangBangController = new BangBangController();

        shootingMode = ShootingMode.IDLE;
    }

    private double getCurrentSpeed() {
        return shooterMotor.getVelocity().getValue();
    }

    private double getTargetSpeed() {
        return switch (shootingMode) {
            case AUTO_SPEAKER -> aimSubsystem.getShooterSetpoint().speed();
            case SPEAKER -> SHOOTER_SPEAKER_SETPOINT.speed();
            case AMP -> SHOOTER_AMP_SETPOINT.speed();
            case TRAP -> SHOOTER_TRAP_SETPOINT.speed();
            case IDLE -> 0;
        };
    }

    private void applyBangBangControl(double targetSpeed) {
        double currentSpeed = getCurrentSpeed();
        double controllerOutput = bangBangController.calculate(currentSpeed, targetSpeed);

        shooterOutputAverager.addMeasurement(controllerOutput);

        shooterMotor.setVoltage(shooterOutputAverager.getValue() * ShooterInfo.SHOOTER_VOLTAGE);
    }

    private void applyIdleSpeed(){
        shooterMotor.set(ShooterInfo.SHOOTER_IDLE_SPEED);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("shooting mode", shootingMode.toString());
        SmartDashboard.putNumber("shooter speed", getCurrentSpeed());
        SmartDashboard.putNumber("shooter target speed", getTargetSpeed());

        switch (shootingMode) {
            case AUTO_SPEAKER, SPEAKER, AMP, TRAP -> applyBangBangControl(getTargetSpeed());
            case IDLE -> applyIdleSpeed();
        }
    }

    public void setShootingMode(ShootingMode shooting){
        shootingMode = shooting;
    }

    @Override
    public ShootingMode getShootingMode() {
        return shootingMode;
    }

    @Override
    public boolean isReady() {
        return isUpToSpeed() && shooterPivot.isAtSetPoint();
    }

    private boolean isUpToSpeed() {
        return Math.abs(getCurrentSpeed()) >= Math.abs(getTargetSpeed()) * 0.95;
    }
}