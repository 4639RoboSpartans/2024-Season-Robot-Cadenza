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

@SuppressWarnings("unused")
public class FalconShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
    private final TalonFX shooterMotor;

    // TODO: extract 10 to a constant
    private final Averager shooterOutput = new Averager(2);
    private final BangBangController bangBangController;

    private boolean isShooterRunning = false;
    private final AimSubsystem aimSubsystem;

    private boolean speakerShooting = true;

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

    public void setSpeaker(){
        double currentSpeed, targetSpeed;
        currentSpeed = getCurrentSpeed();
        ShooterInfo.ShooterSetpoint setpoint = aimSubsystem.getShooterSetpoint();
        targetSpeed = setpoint.speed();
        
        SmartDashboard.putBoolean("speaker shooting", speakerShooting);
        SmartDashboard.putNumber("current speed", currentSpeed);

        double controllerOutput = bangBangController.calculate(currentSpeed, targetSpeed);

        shooterOutput.addMeasurement(controllerOutput);

        shooterMotor.setVoltage(shooterOutput.getValue() * ShooterInfo.SHOOTER_VOLTAGE);
    }

    public void setAmp(){
        double currentSpeed, targetSpeed;
        currentSpeed = getCurrentSpeed();
        targetSpeed = Constants.RobotInfo.ShooterInfo.TARGET_AMP_SHOOTER_SPEED;
        double controllerOutput = bangBangController.calculate(currentSpeed, targetSpeed);
        
        SmartDashboard.putBoolean("speaker shooting", speakerShooting);
        SmartDashboard.putNumber("current speed", currentSpeed);

        shooterMotor.setVoltage(shooterOutput.getValue() * ShooterInfo.SHOOTER_VOLTAGE);
    }

    @Override
    public void periodic() {
        if(!isShooterRunning) {
            shooterMotor.stopMotor();
            return;
        }

        if (speakerShooting)
            setSpeaker();
        else
            setAmp();
    }

    public void setShooting(boolean shooting){
        speakerShooting = shooting;
    }

    private double getCurrentSpeed() {
        return shooterMotor.getVelocity().getValue();
    }

    @Override
    public boolean isUpToSpeed() {
        return Math.abs(getCurrentSpeed()) >= Math.abs(ShooterInfo.TARGET_SPEAKER_SHOOTER_SPEED) * 0.95;
    }

    public void runShooter() {
        isShooterRunning = true;
    }

    public void stopShooter() {
        isShooterRunning = false;
    }
}