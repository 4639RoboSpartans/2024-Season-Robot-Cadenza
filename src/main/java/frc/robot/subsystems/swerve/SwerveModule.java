// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;

    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> anglePosition;
    private StatusSignal<Double> angleVelocity;
    private BaseStatusSignal[] signals;

    private PositionVoltage angleSetter = new PositionVoltage(0);
    //private VelocityTorqueCurrentFOC driveSetter = new VelocityTorqueCurrentFOC(0);
    private VelocityVoltage driveSetter = new VelocityVoltage(0);

    private SwerveModulePosition internalState = new SwerveModulePosition();

    //Creates the Swerve modules Motors and Encoders
    //Relies on the moduleConstants class found in SwerveModuleConstants.java
    /**
     * Create and configure a new swerve module
     * @param moduleNumber Identifies which module it is
     * @param moduleConstants Constants for this module
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Configuration */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "Canivore1");
        configAngleEncoder();

        /* Angle Motor Configuration */
        angleMotor = new TalonFX(moduleConstants.angleMotorID, "Canivore1");
        configAngleMotor(moduleConstants.cancoderID);

        /* Drive Motor Configuration */
        driveMotor = new TalonFX(moduleConstants.driveMotorID, "Canivore1");
        configDriveMotor();

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        anglePosition = angleMotor.getPosition();
        angleVelocity = angleMotor.getVelocity();

        signals = new BaseStatusSignal[4];
        signals[0] = drivePosition;
        signals[1] = driveVelocity;
        signals[2] = anglePosition;
        signals[3] = angleVelocity;
    }

    public void setDesiredState(SwerveModuleState desiredState){
        var optimize = SwerveModuleState.optimize(desiredState, internalState.angle);
        double angleToSet = optimize.angle.getRotations();
        angleMotor.setControl(angleSetter.withPosition(angleToSet));
        double velocityToSet = optimize.speedMetersPerSecond;
        SmartDashboard.putNumber(String.valueOf(moduleNumber) + "Request", velocityToSet);
        driveMotor.setControl(driveSetter.withVelocity(velocityToSet));
    }

    /**
     * Stops the module from driving and turning when called
     */
    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    /**
     * Configure settings for the angle encoder
     */
    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        var angleEncoderConfigs = new CANcoderConfiguration();
        angleEncoderConfigs.MagnetSensor.MagnetOffset = angleOffset;
        angleEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        angleEncoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        angleEncoder.getConfigurator().apply(angleEncoderConfigs);
    }

    /**
     * Configure settings for the steering motor
     */
    private void configAngleMotor(int cancoderID) {
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        var talonfxConfigs = new TalonFXConfiguration();
        talonfxConfigs.Slot0.kV = 0.0;
        talonfxConfigs.Slot0.kS = 0.0;
        talonfxConfigs.Slot0.kP = 40;
        talonfxConfigs.Slot0.kI = 0;
        talonfxConfigs.Slot0.kD = 0.5;
        talonfxConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        talonfxConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        talonfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonfxConfigs.CurrentLimits.SupplyCurrentThreshold = 40;
        talonfxConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
        talonfxConfigs.Voltage.PeakForwardVoltage = 10;
        talonfxConfigs.Voltage.PeakReverseVoltage = -10;
//        talonfxConfigs.Feedback.SensorToMechanismRatio = ModuleConstants.angleGearRatio;
        //talonfxConfigs.Feedback.FeedbackRemoteSensorID = cancoderID;
        //talonfxConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        //talonfxConfigs.Feedback.RotorToSensorRatio = ModuleConstants.angleGearRatio;
        //talonfxConfigs.MotorOutput.NeutralMode = ModuleConstants.angleNeutralMode;
        talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonfxConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        angleMotor.getConfigurator().apply(talonfxConfigs);
        Timer.delay(0.5);
        angleMotor.setPosition(angleEncoder.getAbsolutePosition().refresh().getValue());
    }

    /**
     * Configure settings for the drive motor
     */
    private void configDriveMotor() {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        var talongfxConfigs = new TalonFXConfiguration();
        talongfxConfigs.Slot0.kV = 2.0;
        talongfxConfigs.Slot0.kS = 0.2;
        talongfxConfigs.Slot0.kP = 2;
        talongfxConfigs.Slot0.kI = 0;
        talongfxConfigs.Slot0.kD = 0;
        talongfxConfigs.CurrentLimits.SupplyCurrentLimit = 35;
        talongfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talongfxConfigs.CurrentLimits.SupplyCurrentThreshold = 60;
        talongfxConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
        talongfxConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //talongfxConfigs.MotorOutput.NeutralMode = ModuleConstants.driveNeutralMode;
        talongfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talongfxConfigs.Feedback.SensorToMechanismRatio = 20.52;
        driveMotor.getConfigurator().apply(talongfxConfigs);
        driveMotor.setPosition(0.0);
    }

    /**
     * SwerveModulePosition is an object which contains the modules position and modules angle
     * @return The current position of the module
     */
    public SwerveModulePosition getPosition(boolean refresh) {
        if(refresh) {
            drivePosition.refresh();
            driveVelocity.refresh();
            anglePosition.refresh();
            angleVelocity.refresh();
        }

        double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
        double angleRotations = BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

        double distance = driveRotations / 20.52;
        internalState.distanceMeters = distance;
        Rotation2d angle = Rotation2d.fromRotations(angleRotations);
        internalState.angle = angle;

        return internalState;
    }

    public SwerveModuleState getState(boolean refresh) {
        if(refresh) {
            driveVelocity.refresh();
            anglePosition.refresh();
        }

        return new SwerveModuleState(driveVelocity.getValue(), Rotation2d.fromRotations(anglePosition.getValue()));
    }

    public BaseStatusSignal[] getSignals() {
        return signals;
    }

    public Rotation2d getEncoderAngle() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().refresh().getValue());
    }

    public Rotation2d getMotorAngle() {
        return Rotation2d.fromRotations(angleMotor.getPosition().refresh().getValue());
    }

    public double getMotorSpeed() {
        return driveMotor.getVelocity().refresh().getValue();
    }
}