package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.InterpolatingTables;
import frc.robot.subsystems.SubsystemManager;

public class AimUtil {
    public static Rotation2d getSpeakerRotation(double sidewaysSpeed) {
        return getSpeakerRotation().minus(Rotation2d.fromDegrees(sidewaysSpeed * 5));
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
        return Math.hypot(trans.getX(), trans.getY()) <= 6;
    }

    public static boolean inShootingRange() {
        Translation2d trans = getSpeakerVector();
        return Math.hypot(trans.getX(), trans.getY()) <= 4;
    }

    public static boolean inShootingSector() {
        Translation2d trans = getSpeakerVector();
        double angle = Math.atan(trans.getY() / trans.getX());
        return (angle <= 45 && angle >= 0)
                || (angle >= 315 && angle <= 360)
                || (angle >= -45 && angle <= 0)
                || (angle <= -315 && angle > -360);
    }

    public static boolean aligned() {
        return Math.abs(getSpeakerOffset().getDegrees()) <= 10; //TODO: tune this
    }

    public static Rotation2d getSpeakerRotation() {
        return Rotation2d.fromRadians(MathUtil.clamp(Math.atan(getSpeakerVector().getY() / getSpeakerVector().getX()), -2 * Math.PI, 2 * Math.PI));
    }

    public static Rotation2d getSpeakerOffset() {
        Rotation2d speakerRotation = getSpeakerRotation();
        Rotation2d heading = SubsystemManager.getSwerveDrive().getRotation2d();
        return speakerRotation.minus(heading);
    }

    public static Rotation2d getAmpRotation() {
        Rotation2d ampRot = Rotation2d.fromDegrees(0);
        if (DriverStationUtil.isRed()) {
            ampRot = ampRot.minus(Rotation2d.fromDegrees(90));
        } else {
            ampRot = ampRot.plus(Rotation2d.fromDegrees(90));
        }
        return Rotation2d.fromDegrees(MathUtil.clamp(ampRot.getDegrees(), -360, 360));
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

    public static double[] getShooterSetpoint() {
        Translation2d speakerRelativeBotPose = getSpeakerVector();
        double dist = Math.hypot(speakerRelativeBotPose.getY(), speakerRelativeBotPose.getX());
        double angle = InterpolatingTables.getAngleTable().get(dist);
        double speed = InterpolatingTables.getSpeedTable().get(dist);

        SmartDashboard.putNumber("Shooter Angle: ", angle);
        SmartDashboard.putNumber("Shooter Speed: ", speed);

        return new double[]{speed, angle};
    }
}