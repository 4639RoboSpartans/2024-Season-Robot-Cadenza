package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.RobotInfo.SwerveInfo;
import frc.robot.Constants.SwerveModuleConfig;
import math.MathUtil;

public class SwerveModule {
    private final TalonFX driver, rotator;
    private final CANcoder rotationEncoder;
    private final PIDController rotationPID;
    private final PIDController driverPID;

    private final double rotationOffsetDegrees;
    private final double driveConversionFactor;
    private double targetSpeed = 0;

    public SwerveModule(SwerveModuleConfig swerveModuleData) {
        driveConversionFactor = (1. / 2048) * (1 / 6.55) * (0.1016) * Math.PI;

        driver = switch (Constants.currentRobot) {
            case ZEUS -> new TalonFX(swerveModuleData.driveMotorID());
            case SIREN -> new TalonFX(swerveModuleData.driveMotorID(), "Canivore1");
        };
        rotator = switch (Constants.currentRobot) {
            case ZEUS -> new TalonFX(swerveModuleData.rotatorMotorID());
            case SIREN -> new TalonFX(swerveModuleData.rotatorMotorID(), "Canivore1");
        };
        rotator.setInverted(true);

        driver.setNeutralMode(NeutralModeValue.Brake);
        rotator.setNeutralMode(NeutralModeValue.Brake);

        rotationEncoder = switch(Constants.currentRobot) {
            case ZEUS -> new CANcoder(swerveModuleData.encoderID());
            case SIREN -> new CANcoder(swerveModuleData.encoderID(), "Canivore1");
        };

        rotationOffsetDegrees = swerveModuleData.rotationOffset();

        rotationPID = SwerveInfo.SWERVE_ROTATOR_PID.create(swerveModuleData.rotatorPIDkPMultiplier());
        rotationPID.setTolerance(0.1);
        rotationPID.enableContinuousInput(-180, 180);

        driverPID = SwerveInfo.SWERVE_DRIVER_PID.create(SwerveInfo.K_P_MULTIPLIER); //drive change
        driverPID.setTolerance(0.1); //drive change

        CurrentLimitsConfigs motorCurrentLimiter = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(26)
                .withStatorCurrentLimitEnable(true);

        driver.getConfigurator().apply(motorCurrentLimiter);
        rotator.getConfigurator().apply(motorCurrentLimiter);
    }

    private static boolean isNegligible(SwerveModuleState state) {
        return state.speedMetersPerSecond < 0.0001;
    }

    public void reset() {
        rotationPID.reset();
    }

    public void periodic() {
        int moduleID = rotator.getDeviceID() / 2;
        double currModuleRotation = getRotationInDegrees();

        SmartDashboard.putString("Module %d current rotation".formatted(moduleID), "%.2f degrees".formatted(currModuleRotation));

        double rotatorPIDOutput = rotationPID.calculate(currModuleRotation);
        double driverPIDOutput = - driverPID.calculate(targetSpeed); //drive change

        SmartDashboard.putString("Module %d target speed".formatted(moduleID), "%.2f".formatted(targetSpeed));
        SmartDashboard.putString("Module %d target rotation".formatted(moduleID), "%.2f degrees".formatted(rotationPID.getSetpoint()));
        SmartDashboard.putString("Module %d rotator PID output".formatted(moduleID), "%.2f".formatted(rotatorPIDOutput));
        SmartDashboard.putNumber("Error", driverPID.getVelocityError());

        rotator.set(rotatorPIDOutput);
        driver.set(driverPIDOutput * SwerveInfo.MOVEMENT_SPEED); //drive change-to do: change targetSpeed to driverPIDOutput
    }

    public double getRotationInDegrees() {
        double rawRotationInDegrees = rotationEncoder.getAbsolutePosition().getValue() * 360 - rotationOffsetDegrees;
        return MathUtil.mod(rawRotationInDegrees, -180, 180);
    }

    private void setTargetSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;
    }

    private void setRotation(double degrees) {
        rotationPID.setSetpoint(degrees);
    }

    public double getDriveVelocity() {
        return driver.getVelocity().getValue() * driveConversionFactor;
    }

    public double getDriveDistance() {
        return driver.getPosition().getValue() * driveConversionFactor;
    }

    public double getTurningVelocity() {
        return rotationEncoder.getVelocity().getValue();
    }

    public double getTurningPosition() {
        double rawRotation = rotationEncoder.getAbsolutePosition().getValue() - rotationOffsetDegrees;

        return MathUtil.mod(rawRotation, -180, 180);
    }

    public SwerveModuleState getState() {
        double moduleSpeed = getDriveVelocity();
        double rotationDegrees = getRotationInDegrees();
        Rotation2d rotation = Rotation2d.fromDegrees(rotationDegrees);
        return new SwerveModuleState(moduleSpeed, rotation);
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getRotationInDegrees()));
        setTargetSpeed(optimizedState.speedMetersPerSecond);
        setRotation(optimizedState.angle.getDegrees());
    }

    public void stop() {
        setTargetSpeed(0);
        setRotation(getRotationInDegrees());

        driver.stopMotor();
        rotator.stopMotor();
    }

    public void setBrakeMode(){
        driver.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode(){
        driver.setNeutralMode(NeutralModeValue.Coast);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getTurningPosition()));
    }
}