package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.shooter.ShooterMeasurementLERPer;
import math.Averager;
import math.MathUtil;

import static frc.robot.Constants.RobotInfo.AimInfo;
import static frc.robot.Constants.RobotInfo.ShooterInfo.ShooterSetpoint;
import static frc.robot.Constants.RobotInfo.SwerveInfo;

public class AimSubsystem extends SubsystemBase{

    private final PIDController rotationPID = AimInfo.LIMELIGHT_AIM_PID.create();

    private final Averager angle = new Averager(Constants.POSE_WINDOW_LENGTH);
    private final Averager x = new Averager(Constants.POSE_WINDOW_LENGTH);
    private final Averager z = new Averager(Constants.POSE_WINDOW_LENGTH);
    public AimSubsystem(){}

    public double getRotationSpeed(){
        if (!angle.hasMeasurements()) return 0;

        double yRotation = angle.getValue();

        return rotationPID.calculate(MathUtil.signedPow(yRotation, 0.7)) * SwerveInfo.MAX_ROBOT_SPEED;
    }

    public ShooterSetpoint getShooterSetpoint() {
        ShooterSetpoint result = ShooterMeasurementLERPer.get(x.getValue(), z.getValue());

        SmartDashboard.putNumber("Shooter Angle: ", result.angle());
        SmartDashboard.putNumber("Shooter Speed: ", result.speed());
        
        return result;
    }

    @Override
    public void periodic() {
        double angle = -Math.toRadians(LimeLight.getTx());
        double x = LimeLight.getXDistance();
        double z = LimeLight.getZDistance();

        this.angle.addMeasurement(angle);

        this.x.addMeasurement(x);
        this.z.addMeasurement(z);
    }

    public void resetPID(){
        rotationPID.reset();
    }
}
