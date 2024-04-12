package frc.robot.subsystems.aim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.shooter.ShooterMeasurementLERPer;
import math.Averager;
import math.MathUtil;

import static frc.robot.constants.RobotInfo.AimInfo;
import static frc.robot.constants.RobotInfo.ShooterInfo.LimelightOffsetX;
import static frc.robot.constants.RobotInfo.ShooterInfo.LimelightOffsetZ;
import static frc.robot.constants.RobotInfo.ShooterInfo.ShooterSetpoint;
import static frc.robot.constants.RobotInfo.SwerveInfo;

public class AimSubsystem extends SubsystemBase implements AimInterface {

    private final PIDController rotationPID = AimInfo.LIMELIGHT_AIM_PID_CONSTANTS.create();

    private final Averager angle = new Averager(Constants.POSE_WINDOW_LENGTH);
    private final Averager x = new Averager(Constants.POSE_WINDOW_LENGTH);
    private final Averager z = new Averager(Constants.POSE_WINDOW_LENGTH);
    public AimSubsystem(){
    }

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
        return Math.abs(angle.getValue()) <= AimInfo.AIM_TOLERANCE && LimeLight.seesAprilTags();
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
        double x = LimeLight.getRobotRelativeXDistance();
        double z = LimeLight.getRobotRelativeZDistance();
        double angle = LimeLight.getRobotRelativeXRotation();
        // double angle = LimeLight.getTx();
        double tangent = -Math.atan((x + LimelightOffsetX) / (z  - LimelightOffsetZ));
        if (angle == 0){
            tangent = 0;
            SmartDashboard.putBoolean("sees limelight", false);
        }
        else 
            SmartDashboard.putBoolean("sees limelight", true);
        SmartDashboard.putNumber("atan", tangent);
        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("z", z);
        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber("degrees", Math.toDegrees(angle));

        double distance = Math.hypot(x, z);
        SmartDashboard.putNumber("distance ", distance);

        this.angle.addMeasurement(tangent);

        this.x.addMeasurement(x);
        this.z.addMeasurement(z);

    }

    @Override
    public void resetPID(){
        rotationPID.reset();
    }
}
