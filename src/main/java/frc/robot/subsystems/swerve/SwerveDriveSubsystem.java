// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo;
import frc.robot.network.LimelightHelpers;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemManager;

public class SwerveDriveSubsystem extends SubsystemBase implements ISwerveDriveSubsystem {

    private SwerveModule[] swerveModules;
    private NavX navX;
    private SwerveDrivePoseEstimator poseEstimator;
    BaseStatusSignal[] signals;

    public SwerveDriveSubsystem() {
        swerveModules = new SwerveModule[]{
                new SwerveModule(0, IDs.MODULE_FRONT_LEFT),

                new SwerveModule(1, IDs.MODULE_FRONT_RIGHT),

                new SwerveModule(2, IDs.MODULE_BACK_LEFT),

                new SwerveModule(3, IDs.MODULE_BACK_RIGHT)
        };

        navX = SubsystemManager.getNavX();

        signals = new BaseStatusSignal[16];
        for (int i = 0; i < 4; i++) {
            BaseStatusSignal[] tempSignals = swerveModules[i].getSignals();
            signals[i * 4 + 0] = tempSignals[0];
            signals[i * 4 + 1] = tempSignals[1];
            signals[i * 4 + 2] = tempSignals[2];
            signals[i * 4 + 3] = tempSignals[3];
        }
    }

    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        poseEstimator.update(navX.getRotation2d(), getPositions(true));
        Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        if (limelightPose.getX() != 0 && limelightPose.getY() != 0) {
            poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp());
        }
        if (DriverStation.isDisabled()) {
            lock();
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(navX.getRotation2d(), getPositions(true), pose);
    }

    /**
     * Main controlling method for driving swerve based on desired speed of drivetrian
     *
     * @param chassisSpeeds Desired speed of drivetrain
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = RobotInfo.SwerveInfo.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }


    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 5.0);
        swerveModules[0].setDesiredState(states[0]);
        swerveModules[1].setDesiredState(states[1]);
        swerveModules[2].setDesiredState(states[2]);
        swerveModules[3].setDesiredState(states[3]);
    }


    @Override
    public Rotation2d getRotation2d() {
        return navX.getRotation2d();
    }

    @Override
    public void setMovement(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, navX.getRotation2d());
        drive(robotRelative);
    }

    @Override
    public void setRawMovement(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds);
    }

    public void stop() {
        swerveModules[0].stop();
        swerveModules[1].stop();
        swerveModules[2].stop();
        swerveModules[3].stop();
    }


    public void lock() {
        swerveModules[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
        swerveModules[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
        swerveModules[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
        swerveModules[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    }


    public SwerveModulePosition[] getPositions(boolean refresh) {
        return new SwerveModulePosition[]{
                swerveModules[0].getPosition(refresh),
                swerveModules[1].getPosition(refresh),
                swerveModules[2].getPosition(refresh),
                swerveModules[3].getPosition(refresh)
        };
    }
}