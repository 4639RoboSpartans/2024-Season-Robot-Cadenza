package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotInfo.ShooterInfo;
import frc.robot.Constants.RobotInfo.ShooterInfo.ShooterSetpoint;

public class ShooterMeasurementLERPer {
    public static ShooterSetpoint get(double dx, double dz) {
        double distance = Math.hypot(dx, dz);

        SmartDashboard.putNumber("Limelight distance: ", distance);

        // Lerp
        return ShooterInfo.measurements[0].setpoint();
    }
}
