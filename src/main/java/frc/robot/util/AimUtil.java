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
    public static Translation2d getSpeakerPose() {
        Translation2d speakerPose;
        if (DriverStationUtil.isRed()) {
            speakerPose = FieldConstants.speakerPose_red;
        } else {
            speakerPose = FieldConstants.speakerPose_blue;
        }
        return speakerPose;
    }

    public static Translation2d getAmpPose() {
        Translation2d ampPose;
        if (DriverStationUtil.isRed()) {
            ampPose = FieldConstants.ampPose_red;
        } else {
            ampPose = FieldConstants.ampPose_blue;
        }
        return ampPose;
    }

    public static Rotation2d getSpeakerRotation(double sidewaysSpeed) {
        return getSpeakerRotation().minus(Rotation2d.fromDegrees(sidewaysSpeed * 5));
    }

    public static Translation2d getPoseVector(Translation2d targetTranslation) {
        Pose2d currBotPose = SubsystemManager.getSwerveDrive().getPose();
        Translation2d currBotTranslation = currBotPose.getTranslation();
        return currBotTranslation.minus(targetTranslation);
    }

    public static Rotation2d getPoseRotation(Translation2d targetTranslation) {
        return Rotation2d.fromRadians(
                MathUtil.clamp(
                        Math.atan(
                                getPoseVector(targetTranslation).getY()
                                / getPoseVector(targetTranslation).getX()
                        ),
                        -2 * Math.PI,
                        2 * Math.PI
                )
        );
    }

    public static Translation2d getSpeakerVector() {
        return getPoseVector(getSpeakerPose());
    }

    public static Rotation2d getSpeakerRotation() {
        return getPoseRotation(getSpeakerPose());
    }

    public static boolean inRange() {
        Translation2d trans = getSpeakerVector();
        return Math.hypot(trans.getX(), trans.getY()) <= 4;
    }

    public static boolean inShootingRange() {
        Translation2d trans = getSpeakerVector();
        return Math.hypot(trans.getX(), trans.getY()) <= 3;
    }

    public static boolean inShootingSector() {
        Rotation2d rotation = getSpeakerRotation();
        return Math.abs(rotation.getDegrees()) <= 30;
    }

    public static boolean aligned() {
        return Math.abs(getSpeakerOffset().getDegrees()) <= 3.5; //TODO: tune this
    }

    public static Rotation2d getSpeakerOffset() {
        Rotation2d speakerRotation = getSpeakerRotation();
        Rotation2d heading = SubsystemManager.getSwerveDrive().getRotation2d();
        return Rotation2d.fromDegrees(MathUtil.clamp(
            speakerRotation.minus(heading).getDegrees(), -360, 360));
    }

    public static Rotation2d getAmpRotation() {
        return Rotation2d.fromDegrees(90);
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