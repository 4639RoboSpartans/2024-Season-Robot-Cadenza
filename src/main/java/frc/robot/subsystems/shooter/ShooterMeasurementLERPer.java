package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotInfo.ShooterInfo;
import frc.robot.constants.ShooterSetpoint;
import frc.robot.constants.ShooterSetpointMeasurement;
import math.MathUtil;

public class ShooterMeasurementLERPer {
  public static ShooterSetpoint get(double dx, double dz) {
    double distance = Math.hypot(dx, dz);

    SmartDashboard.putNumber("Limelight distance: ", distance);

    ShooterSetpointMeasurement a = ShooterInfo.measurements[0], b = ShooterInfo.measurements[1];
    for (int i = 1; i < ShooterInfo.measurements.length; i++) {
      a = ShooterInfo.measurements[i - 1];
      b = ShooterInfo.measurements[i];
      if (a.distance() < distance && distance < b.distance()) {
        break;
      }
    }

    // Lerp
    double t = (distance - a.distance()) / (b.distance() - a.distance());
    t = MathUtil.clamp(t, 0, 1);

    return new ShooterSetpoint(
        MathUtil.lerp(a.setpoint().speed(), b.setpoint().speed(), t),
        MathUtil.lerp(a.setpoint().angle(), b.setpoint().angle(), t));

    //        return ShooterInfo.measurements[2].setpoint();
  }
}
