package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.ShooterMeasurementLERPer;

public class AimUtil {
    public static Translation2d getSpeakerRelativeMovement(double forwardsSpeed, double sidewaysSpeed) {
        Rotation2d speakerRotation = getSpeakerRotation();
        Rotation2d movementRotation = Rotation2d.fromRadians(Math.atan(sidewaysSpeed / forwardsSpeed));
        double totalMovement = Math.hypot(forwardsSpeed, sidewaysSpeed);
        Rotation2d rotation = movementRotation.minus(speakerRotation);
        return new Translation2d(
          Math.sin(rotation.getRadians()) * totalMovement,
          Math.cos(rotation.getRadians()) * totalMovement
        );
    }

    public static Rotation2d getSpeakerRotation(double forwardsSpeed, double sidewaysSpeed) {
        Translation2d speakerRelativeMovement = getSpeakerRelativeMovement(forwardsSpeed, sidewaysSpeed);
        return getSpeakerRotation().plus(Rotation2d.fromDegrees(speakerRelativeMovement.getX() * 1));//TODO: tune scalar
    }

    public static Translation2d getSpeakerVector() {
        Pose2d currBotPose = SubsystemManager.getSwerveDrive().getPose();
        Translation2d currBotTranslation = currBotPose.getTranslation();
        Translation2d speakerPose;
        if (DriverStationUtil.isRed()) {
            speakerPose = FieldConstants.speakerPose_red;
        } else {
            speakerPose = FieldConstants.speakerPose_blue;
        }
        return currBotTranslation.minus(speakerPose);
    }


    public static boolean inRange() {
        Translation2d trans = getSpeakerVector();
        return Math.hypot(trans.getX(), trans.getY()) <= 5;
    }

    public static boolean inShootingRange() {
        Translation2d trans = getSpeakerVector();
        return Math.hypot(trans.getX(), trans.getY()) <= 3;//TODO: tune this
    }

    public static boolean inShootingSector() {
        return Math.abs(getSpeakerRotation().getDegrees()) < 45;
    }

    public static boolean aligned() {
        Rotation2d speakerRotation = getSpeakerRotation();
        Rotation2d heading = SubsystemManager.getSwerveDrive().getRotation2d();
        Rotation2d diff = speakerRotation.minus(heading);
        return Math.abs(diff.getDegrees()) < 10;//TODO: tune this
    }

    public static boolean inShootingRange() {
        Translation2d trans = getSpeakerVector();
        return Math.hypot(trans.getX(), trans.getY()) <= 5;
    }

    public static Rotation2d getSpeakerRotation() {
        return Rotation2d.fromRadians(Math.atan(getSpeakerVector().getY() / getSpeakerVector().getX()) % (2 * Math.PI));
    }

    public static Rotation2d getAmpRotation(double forwardsSpeed, double sidewaysSpeed) {
        Translation2d ampVector = getAmpVector();
        Translation2d botVector = new Translation2d(sidewaysSpeed, forwardsSpeed);
        Translation2d shootingVector = getShootingVector(ampVector, botVector);
        return new Rotation2d(Math.atan(shootingVector.getY() / shootingVector.getX()));
    }

    public static Translation2d getAmpVector() {
        Pose2d currBotPose = SubsystemManager.getSwerveDrive().getPose();
        Translation2d currBotTranslation = currBotPose.getTranslation();
        Translation2d ampPose;
        if (DriverStationUtil.isRed()) {
            ampPose = FieldConstants.ampPose_red;
        } else {
            ampPose = FieldConstants.ampPose_blue;
        }
        return currBotTranslation.minus(ampPose);
    }

    private static Translation2d getShootingVector(
            Translation2d noteVector, Translation2d botVector) {
        return noteVector.minus(botVector);
    }

    private static RobotInfo.ShooterInfo.ShooterSetpoint getShooterSetpoint(double dist) {
        Translation2d speakerRelativeBotPose = getSpeakerVector();
        RobotInfo.ShooterInfo.ShooterSetpoint result =
                ShooterMeasurementLERPer.get(dist);

        SmartDashboard.putNumber("Shooter Angle: ", result.angle());
        SmartDashboard.putNumber("Shooter Speed: ", result.speed());

        return result;
    }

    public static RobotInfo.ShooterInfo.ShooterSetpoint getVelocityCompensatedShooterSetpoint() {
        Translation2d speakerRelativeMovement = getSpeakerRelativeMovement(
                Controls.DriverControls.SwerveForwardAxis.getAsDouble(),
                Controls.DriverControls.SwerveStrafeAxis.getAsDouble()
                );
        return getShooterSetpoint(speakerRelativeMovement.getY() / 5);//TODO: tune this
    }
}