package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConfig;
import math.math;

public class SwerveModule {
    private final TalonFX driver, rotator;
    private final CANcoder rotationEncoder;
    private final PIDController rotationPID;

    private final double rotationOffset;
    
    private final double kp = 0.005;
    private final double ki = 0;
    private final double kd = 0;

    private double speed = 0;

    private final double DriveConversionFactor;

    public SwerveModule(SwerveModuleConfig swerveModuleData){
        DriveConversionFactor = (1. /2048)*(1/6.55)*(0.1016)*Math.PI;

        driver = new TalonFX(swerveModuleData.driveMotorID());
        rotator = new TalonFX(swerveModuleData.rotaterMotorID());

        driver.setNeutralMode(NeutralModeValue.Coast);
        rotator.setNeutralMode(NeutralModeValue.Coast);

        rotationEncoder = new CANcoder(swerveModuleData.encoderID());

        rotationOffset = swerveModuleData.rotationOffset();

        rotationPID = new PIDController(
            Constants.RobotInfo.ROTATOR_MOTOR_KP,
            Constants.RobotInfo.ROTATOR_MOTOR_KI,
            0
        );
        rotationPID.setTolerance(0.1);
        rotationPID.enableContinuousInput(-180, 180);

    }

    public void periodic() {
        rotator.setVoltage(rotationPID.calculate(getRotationInDegrees()));
        driver.set(speed * Constants.RobotInfo.MOVEMENT_SPEED);

        // SmartDashboard.putNumber("Module Speed " + driver.getDeviceID(), speed);
        // SmartDashboard.putNumber("Module Rotation Encoder " + driver.getDeviceID(), getRotationInDegrees());
        // SmartDashboard.putNumber("Module Rotation Setpoint " + driver.getDeviceID(), rotationPID.getSetpoint());
        // SmartDashboard.putNumber("Module Rotation Voltage"+rotator.getDeviceID(),    -rotationPID.calculate(getRotationInDegrees()));
        // SmartDashboard.putNumber("Module Rotation Value " + driver.getDeviceID(), rotate);
    }

    public double getRotationInDegrees(){
        double rawRotation = rotationEncoder.getAbsolutePosition().getValue() - rotationOffset;
        return math.mod(rawRotation, -180, 180);
    }

    private void setSpeed(double speed){
        this.speed = speed;
    }

    private void setRotation(double degrees){
        rotationPID.setSetpoint(degrees);
    }

    public double getDriveVelocity(){
        return driver.getVelocity().getValue() * DriveConversionFactor;
    }
    public double getDriveDistance(){
        return driver.getPosition().getValue() * DriveConversionFactor;
    }

    public double getTurningVelocity(){
        return rotationEncoder.getVelocity().getValue();
    }
    public double getTurningPosition(){
        double rawRotation = rotationEncoder.getAbsolutePosition().getValue() - rotationOffset;

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
            // SwerveModuleState optimizedState = SwerveUtil.optimize(state, getRotationInDegrees());
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