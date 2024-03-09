package frc.robot.subsystems.aim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.shooter.ShooterMeasurementLERPer;
import math.Averager;
import math.MathUtil;

import static frc.robot.constants.Constants.RobotInfo.AimInfo;
import static frc.robot.constants.Constants.RobotInfo.ShooterInfo.ShooterSetpoint;
import static frc.robot.constants.Constants.RobotInfo.SwerveInfo;

public class AimSubsystem extends SubsystemBase implements AimInterface {

    private final PIDController rotationPID = AimInfo.LIMELIGHT_AIM_PID_CONSTANTS.create();

    private final Averager angle = new Averager(Constants.POSE_WINDOW_LENGTH);
    private final Averager x = new Averager(Constants.POSE_WINDOW_LENGTH);
    private final Averager z = new Averager(Constants.POSE_WINDOW_LENGTH);
    public AimSubsystem(){}

    @Override
    public double getSwerveRotation(){
        if (!angle.hasMeasurements()) return 0;

        double yRotation = angle.getValue();
        yRotation *= (1 + getTxDerivative() * SwerveInfo.DERIVATIVE_MULTIPLIER);

        return rotationPID.calculate(MathUtil.signedPow(yRotation, AimInfo.AIM_ROT_POW)) * SwerveInfo.AIM_ROTATION_SPEED;
    }

    @Override
    public double getTxDerivative(){
        return x.getDerivative();
    }

    @Override
    public boolean isAtSetpoint() {
        return Math.abs(angle.getValue()) <= AimInfo.AIM_TOLERANCE;
    }

    @Override
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

    @Override
    public void resetPID(){
        rotationPID.reset();
    }
}
