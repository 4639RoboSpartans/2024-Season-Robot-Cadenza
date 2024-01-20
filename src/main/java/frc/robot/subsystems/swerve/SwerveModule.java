package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConfig;
import math.math;

public class SwerveModule {
    private final TalonFX driver, rotator;
    private final CANcoder rotationEncoder;
    private final PIDController rotationPID;

    private final double rotationOffsetDegrees;

    private double speed = 0;

    private final double driveConversionFactor;

    private CurrentLimitsConfigs motorCurrentLimiter;

    public SwerveModule(SwerveModuleConfig swerveModuleData){
        driveConversionFactor = (1./2048)*(1/6.55)*(0.1016)*Math.PI;

        driver = new TalonFX(swerveModuleData.driveMotorID());
        rotator = new TalonFX(swerveModuleData.rotatorMotorID());
        rotator.setInverted(true);

        driver.setNeutralMode(NeutralModeValue.Coast);
        rotator.setNeutralMode(NeutralModeValue.Coast);

        rotationEncoder = new CANcoder(swerveModuleData.encoderID());

        rotationOffsetDegrees = swerveModuleData.rotationOffset();

        rotationPID = Constants.RobotInfo.SWERVE_ROTATOR_PID.create(swerveModuleData.rotatorPIDkPMultiplier());
        rotationPID.setTolerance(0.1);
        rotationPID.enableContinuousInput(-180, 180);

        motorCurrentLimiter = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(26)
            .withStatorCurrentLimitEnable(true);

        driver.getConfigurator().apply(motorCurrentLimiter);
        rotator.getConfigurator().apply(motorCurrentLimiter);
    }

    public void reset() {
        rotationPID.reset();
    }

    public void periodic() {
        double rotation = getRotationInDegrees();
        double rotationSpeed = rotationPID.calculate(rotation);

        int moduleID = rotator.getDeviceID() / 2;
        SmartDashboard.putString(moduleID + " rotation", "%.2f".formatted(rotation));
        SmartDashboard.putString(moduleID + " rotator PID output", "%.2f".formatted(rotationSpeed));

        rotator.set(rotationSpeed);
        driver.set(speed * Constants.RobotInfo.MOVEMENT_SPEED);
    }

    public double getRotationInDegrees(){
        double rawRotationInDegrees = rotationEncoder.getAbsolutePosition().getValue() * 360 - rotationOffsetDegrees;
        return math.mod(rawRotationInDegrees, -180, 180);
    }

    private void setSpeed(double speed){
        this.speed = speed;
    }

    private void setRotation(double degrees){
        rotationPID.setSetpoint(degrees);
    }

    public double getDriveVelocity(){
        return driver.getVelocity().getValue() * driveConversionFactor;
    }
    public double getDriveDistance(){
        return driver.getPosition().getValue() * driveConversionFactor;
    }

    public double getTurningVelocity(){
        return rotationEncoder.getVelocity().getValue();
    }
    public double getTurningPosition(){
        double rawRotation = rotationEncoder.getAbsolutePosition().getValue() - rotationOffsetDegrees;

        return math.mod(rawRotation, -180, 180);
    }

    public SwerveModuleState getState() {
        double moduleSpeed = getDriveVelocity();
        double rotationDegrees = getRotationInDegrees();
        Rotation2d rotation = Rotation2d.fromDegrees(rotationDegrees);
        return new SwerveModuleState(moduleSpeed, rotation);
    }

    public void setState(SwerveModuleState state){
        if(isNegligible(state)) stop();
        else{
            SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getRotationInDegrees()));
            setSpeed(optimizedState.speedMetersPerSecond);
            setRotation(optimizedState.angle.getDegrees());
        }
    }

    public void resetAngleAndPosition(){
        setSpeed(0);
        setRotation(0);
    }

    public void stop(){
        setSpeed(0);
        setRotation(getRotationInDegrees());

        driver.stopMotor();
        rotator.stopMotor();
    }
    
    private static boolean isNegligible(SwerveModuleState state){
        return state.speedMetersPerSecond < 0.001;
    }
}