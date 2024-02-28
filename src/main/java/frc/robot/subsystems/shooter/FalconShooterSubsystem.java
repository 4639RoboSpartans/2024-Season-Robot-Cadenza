package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.AimSubsystem;
import math.Averager;

import static frc.robot.Constants.RobotInfo.*;
import static frc.robot.Constants.RobotInfo.ShooterInfo.*;

@SuppressWarnings("unused")
public class FalconShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
    private final TalonFX shooterMotor;

    // TODO: extract 10 to a constant
    private final Averager shooterOutput = new Averager(2);
    private final BangBangController bangBangController;

    private boolean isShooting = false;
    private final AimSubsystem aimSubsystem;

    private SHOOTING_MODE shootingMode = SHOOTING_MODE.SPEAKER;

    public FalconShooterSubsystem(int shooterMotorID, AimSubsystem aimSubsystem) {
        shooterMotor = new TalonFX(shooterMotorID);
        this.aimSubsystem = aimSubsystem;

        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor.setInverted(true);

        shooterMotor.getConfigurator().apply(
            new CurrentLimitsConfigs().withStatorCurrentLimit(3)
        );

        bangBangController = new BangBangController();
    }

    private void setSpeakerSpeed(){
        double currentSpeed, targetSpeed;
        currentSpeed = getCurrentSpeed();
        ShooterInfo.ShooterSetpoint setpoint = aimSubsystem.getShooterSetpoint();
        targetSpeed = setpoint.speed();
        double controllerOutput = bangBangController.calculate(currentSpeed, targetSpeed);

        shooterOutput.addMeasurement(controllerOutput);

        shooterMotor.setVoltage(shooterOutput.getValue() * ShooterInfo.SHOOTER_VOLTAGE);
    }

    private void setAmpSpeed(){
        double currentSpeed, targetSpeed;
        currentSpeed = getCurrentSpeed();
        targetSpeed = Constants.RobotInfo.ShooterInfo.TARGET_AMP_SHOOTER_SPEED;

        double controllerOutput = bangBangController.calculate(currentSpeed, targetSpeed);

        shooterOutput.addMeasurement(controllerOutput);

        shooterMotor.setVoltage(shooterOutput.getValue() * ShooterInfo.SHOOTER_VOLTAGE);
    }

    private void setIdleSpeed(){
        double currentSpeed, targetSpeed;
        targetSpeed = Constants.RobotInfo.ShooterInfo.TARGET_IDLE_SHOOTER_SPEED;
        shooterMotor.set(targetSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("shooting", isShooting);
        SmartDashboard.putNumber("current speed", getCurrentSpeed());
        if(!isShooting) {
            setIdleSpeed();
        }
        else {
            if (shootingMode == SHOOTING_MODE.SPEAKER){
                setSpeakerSpeed();
            }
            else{
                setAmpSpeed();
            }
        }
    }

    public void setShooting(SHOOTING_MODE shooting){
        shootingMode = shooting;
    }

    private double getCurrentSpeed() {
        return shooterMotor.getVelocity().getValue();
    }

    @Override
    public boolean isUpToSpeed() {
        return Math.abs(getCurrentSpeed()) >= Math.abs(ShooterInfo.TARGET_SPEAKER_SHOOTER_SPEED) * 0.95;
    }

    public void runShooter() {
        isShooting = true;
    }

    public void stopShooter() {
        isShooting = false;
    }
}