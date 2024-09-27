package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.InterpolatingTables;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.pivot.IShooterPivotSubsystem;
import frc.robot.util.AimUtil;
import math.Averager;

import static frc.robot.constants.RobotInfo.ShooterInfo;
import static frc.robot.constants.RobotInfo.ShooterInfo.*;

public class FalconShooterFeedforward extends SubsystemBase implements IShooterSubsystem {
    // Components
    private final TalonFX shooterMotorLeft;
    private final TalonFX shooterMotorRight;
    private final IShooterPivotSubsystem shooterPivot;
    private final PIDController speedFeedback;

    private final SimpleMotorFeedforward speedFeedforward;
    // State
    private ShootingMode shootingMode;

    public FalconShooterFeedforward(int shooterMotorLeftID, int shooterMotorRightID) {
        this.shooterPivot = SubsystemManager.getShooterPivot(this);

        shooterMotorLeft = new TalonFX(shooterMotorLeftID, "Canivore1");
        shooterMotorRight = new TalonFX(shooterMotorRightID, "Canivore1");
        shooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorRight.setNeutralMode(NeutralModeValue.Coast);

        shooterMotorLeft.setInverted(true);
        shooterMotorLeft.getConfigurator().apply(
                new CurrentLimitsConfigs().withStatorCurrentLimit(4)
        );
        shooterMotorRight.getConfigurator().apply(
                new CurrentLimitsConfigs().withStatorCurrentLimit(4)
        );
        shooterMotorRight.setControl(new Follower(shooterMotorLeftID, true));

        speedFeedback = new PIDController(ShooterKp, ShooterKi,  ShooterKd);

        speedFeedforward = new SimpleMotorFeedforward(0, ShooterKv, 0);

        shootingMode = ShootingMode.IDLE;
    }

    private double getCurrentSpeed() {
        return shooterMotorLeft.getVelocity().getValue();
    }

    private double getTargetSpeed() {
        return switch (shootingMode) {
            case AUTO_SPEAKER, SPINUP -> AimUtil.getShooterSetpoint()[0];
            case SPEAKER -> SHOOTER_SPEAKER_SETPOINT.speed();
            case AMP -> SHOOTER_AMP_SETPOINT.speed();
            case IDLE, INTAKE -> 0;
            case LAUNCH -> SHOOTER_LAUNCH_SETPOINT.speed();
        };
    }

    private void applyBangBangControl(double targetSpeed) {
        double currentSpeed = getCurrentSpeed();
        double feedbackOutput = speedFeedback.calculate(currentSpeed, targetSpeed);
        double feedForwardOutput = speedFeedforward.calculate(currentSpeed, targetSpeed);

        shooterMotorLeft.setVoltage(feedbackOutput + feedForwardOutput);
    }

    private void applyIdleSpeed() {
        double speed = Robot.isInAuton()
                ? SHOOTER_AUTON_IDLE_SPEED
                : SHOOTER_IDLE_SPEED;
        shooterMotorLeft.set(speed);
    }

    private void applyIntakeSpeed() {
        shooterMotorLeft.set(ShooterInfo.SHOOTER_INTAKE_SPEED);
    }

    @Override
    public void periodic() {
        InterpolatingTables.update();
        switch (shootingMode) {
            case AUTO_SPEAKER, SPEAKER, AMP, LAUNCH, SPINUP -> applyBangBangControl(getTargetSpeed());
            case IDLE -> applyIdleSpeed();
            case INTAKE -> applyIntakeSpeed();
        }
        SmartDashboard.putNumber("Shooter/shooter target speed", getTargetSpeed());
        SmartDashboard.putNumber("Shooter/shooter measured speed", getCurrentSpeed());
        SmartDashboard.putString("Shooter/shooting mode", shootingMode.toString());
    }

    public void setShootingMode(ShootingMode shooting) {
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
        return Math.abs(getCurrentSpeed()) >= Math.abs(getTargetSpeed()) * switch (shootingMode) {
            case AMP -> .98;
            default -> 0.95;
        };
    }
}