package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotInfo.ShooterInfo.ShooterSetpoint;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.ShooterMeasurementLERPer;

public class AimUtil {
  public static Translation2d getSpeakerToRobotVector() {
    Pose2d currBotPose = SubsystemManager.getSwerveDrive().getPose();
    Translation2d currBotTranslation = currBotPose.getTranslation();
    Translation2d speakerPose;
    if (SmartDashboard.getBoolean("Alliance", false)) {
      speakerPose = FieldConstants.speakerPose_red;
    } else {
      speakerPose = FieldConstants.speakerPose_blue;
    }
    return currBotTranslation.minus(speakerPose);
  }

  public static ShooterSetpoint getShooterSetpoint() {
    Translation2d SpeakerRelativeBotPose = getSpeakerToRobotVector();
    double x = SpeakerRelativeBotPose.getX();
    double y = SpeakerRelativeBotPose.getY();
    return ShooterMeasurementLERPer.get(x, y);
  }
}
