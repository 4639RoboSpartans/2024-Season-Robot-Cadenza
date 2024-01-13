package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.subsystems.NavX;

import math.math;

public class SwerveDriveSubsystem extends SubsystemBase {
    public Field2d field;

    private final SwerveModule
            moduleFrontLeft,
            moduleFrontRight,
            moduleBackLeft,
            moduleBackRight;

    public final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final NavX navx;
    private final double wheelRadius;

    private Pose2d pose;

    private final PIDController PID;
    private final double kp;
    private final double ki;
    private final double kd;
    //blue side bottom node is 0,0 on the field
    public SwerveDriveSubsystem(NavX navx){
        field = new Field2d();
        SmartDashboard.putData(field);
        moduleFrontLeft  = new SwerveModule(Constants.IDs.MODULE_FRONT_LEFT);
        moduleFrontRight = new SwerveModule(Constants.IDs.MODULE_FRONT_RIGHT);
        moduleBackLeft   = new SwerveModule(Constants.IDs.MODULE_BACK_LEFT);
        moduleBackRight  = new SwerveModule(Constants.IDs.MODULE_BACK_RIGHT);

        kp =0.12;
        ki =0.1;
        kd = 0;
        PID = new PIDController(kp, ki, kd);

        this.navx = navx;
        pose = new Pose2d();

        wheelRadius = Constants.RobotInfo.robotBaseLength / 2;
        kinematics = Constants.RobotInfo.DriveConstants.kDriveKinematics;

        // Creating my odometry object from the kinematics object and the initial wheel positions.
        // Here, our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing the opposing alliance wall.
        // if(AprilTagDetected()){
        //     double Xoffset = 0;//subject to change
        //     double Yoffset = 0;//subject to change
        //     double tempx = FieldD3Coords()[0];
        //     double tempy = FieldD3Coords()[1];
        //     double tempRot = FieldD3Coords()[2];
        //     odometry = new SwerveDriveOdometry(
        //         kinematics,
        //         navx.getGyroRotation2d(),
        //         getSwerveModulePositions(),
        //         new Pose2d(tempx+Xoffset, tempy+Yoffset, new Rotation2d(Math.PI))
        //     );
        // }else{

        odometry = new SwerveDriveOdometry(
                kinematics,
                navx.getGyroRotation2d(),
                getSwerveModulePositions(),
                new Pose2d(0, 0, new Rotation2d(0))
                // new Pose2d(16.48, 8.1, new Rotation2d(navx.getHeading()+Math.PI))
        );

    }
    // }

    public void setMovement(double horizontalOffset, double angle){
        double voltage = PID.calculate(horizontalOffset, 0);
        double speed = voltage/12.5;

        double speedFrontLeft  = speed;
        double speedFrontRight = speed;
        double speedBackLeft   = speed;
        double speedBackRight  = speed;

        double angleFrontLeft  = angle;
        double angleFrontRight = angle;
        double angleBackLeft   = angle;
        double angleBackRight  = angle;

        double max = Math.max(
                Math.max(
                    speedFrontLeft,
                    speedFrontRight
                ),
                Math.max(
                    speedBackLeft,
                    speedBackRight
                )
        );

        if (max > 1) {
            speedFrontLeft  /= max;
            speedFrontRight /= max;
            speedBackLeft   /= max;
            speedBackRight  /= max;
        }

        SmartDashboard.putNumber("angle 1", angleFrontLeft);
        SmartDashboard.putNumber("angle 2", angleFrontRight);
        SmartDashboard.putNumber("angle 3", angleBackLeft);
        SmartDashboard.putNumber("angle 4", angleBackRight);

        setModulesStates(
                new SwerveModuleState(speedFrontLeft, Rotation2d.fromDegrees(angleFrontLeft)),
                new SwerveModuleState(speedFrontRight, Rotation2d.fromDegrees(angleFrontRight)),
                new SwerveModuleState(speedBackLeft, Rotation2d.fromDegrees(angleBackLeft)),
                new SwerveModuleState(speedBackRight, Rotation2d.fromDegrees(angleBackRight))
        );
    }

    public double getHeading(){
        return navx.getHeading();
    }

    public Pose2d getFieldPos(){
        return odometry.getPoseMeters();
    }

    public void setMovement(SwerveMovement swerveMovement){
        double A = swerveMovement.forwardMovement() - swerveMovement.rotation();
        double B = swerveMovement.forwardMovement() + swerveMovement.rotation();
        double C = swerveMovement.strideMovement()  + swerveMovement.rotation();
        double D = swerveMovement.strideMovement()  - swerveMovement.rotation();

        double speedFrontLeft  = math.magnitude(A, C);
        double speedFrontRight = math.magnitude(B, C);
        double speedBackLeft   = math.magnitude(A, D);
        double speedBackRight  = math.magnitude(B, D);

        double angleFrontLeft  = math.atan(C, A);
        double angleFrontRight = math.atan(C, B);
        double angleBackLeft   = math.atan(D, A);
        double angleBackRight  = math.atan(D, B);

        double max = math.max(
                speedFrontLeft,
                speedFrontRight,
                speedBackLeft,
                speedBackRight
        );

        if (max > 1) {
            speedFrontLeft  /= max;
            speedFrontRight /= max;
            speedBackLeft   /= max;
            speedBackRight  /= max;
        }

        SmartDashboard.putNumber("angle 1", angleFrontLeft);
        SmartDashboard.putNumber("angle 2", angleFrontRight);
        SmartDashboard.putNumber("angle 3", angleBackLeft);
        SmartDashboard.putNumber("angle 4", angleBackRight);

        setModulesStates(
                new SwerveModuleState(speedFrontLeft, Rotation2d.fromDegrees(angleFrontLeft)),
                new SwerveModuleState(speedFrontRight, Rotation2d.fromDegrees(angleFrontRight)),
                new SwerveModuleState(speedBackLeft, Rotation2d.fromDegrees(angleBackLeft)),
                new SwerveModuleState(speedBackRight, Rotation2d.fromDegrees(angleBackRight))
        );
    }

    public void setModulesStatess(SwerveModuleState[] states){
        setModulesStates(states[0], states[1], states[2], states[3]);
    }

    public void setModulesStates(
            SwerveModuleState stateFrontLeft,
            SwerveModuleState stateFrontRight,
            SwerveModuleState stateBackLeft,
            SwerveModuleState stateBackRight
    ){

        moduleFrontLeft .setState(stateFrontLeft);
        moduleFrontRight.setState(stateFrontRight);
        moduleBackLeft  .setState(stateBackLeft);
        moduleBackRight .setState(stateBackRight);

        SmartDashboard.putString("FL state", stateFrontLeft.toString());
        SmartDashboard.putString("FR state", stateFrontRight.toString());
        SmartDashboard.putString("BL state", stateBackLeft.toString());
        SmartDashboard.putString("BR state", stateBackRight.toString());
    }

    public void setModulesStates(SwerveModuleState state){
        setModulesStates(state, state, state, state);
    }

    public void stop(){
        setModulesStates(new SwerveModuleState());
    }



    public void resetPose() {
        odometry.resetPosition(
                navx.getGyroRotation2d(),
                getSwerveModulePositions(),
                new Pose2d(0, 0, new Rotation2d())
        );
    }

    public void resetOdometry(Pose2d pos){
        odometry.resetPosition(getRotation(), getSwerveModulePositions(), pos);
    }

    public void setPose(double x, double y){
        odometry.resetPosition(
                navx.getGyroRotation2d(),
                getSwerveModulePositions(),
                new Pose2d(x, y, new Rotation2d())
        );
    }
    //width 26 ft 7 inches length 54 ft 1 inch
    // 8.1 meters by 16.48 meters
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                new SwerveModulePosition(-moduleFrontLeft.getDriveDistance(), new Rotation2d(Math.toRadians(moduleFrontLeft.getRotationInDegrees()))),
                new SwerveModulePosition(-moduleBackLeft.getDriveDistance(), new Rotation2d(Math.toRadians(moduleBackLeft.getRotationInDegrees()))),
                new SwerveModulePosition(-moduleFrontRight.getDriveDistance(), new Rotation2d(Math.toRadians(moduleFrontRight.getRotationInDegrees()))),

                new SwerveModulePosition(-moduleBackRight.getDriveDistance(), new Rotation2d(Math.toRadians(moduleBackRight.getRotationInDegrees())))
        };
    }

    public Rotation2d getRotation() {
        return navx.getGyroRotation2d();
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(navx.getHeading());
    }

    @Override
    public void periodic() {

        moduleFrontLeft.periodic();
        moduleFrontRight.periodic();
        moduleBackLeft.periodic();
        moduleBackRight.periodic();

        // Get the rotation of the robot from the gyro.


        // SmartDashboard.putNumber("Gyro angle", navx.getHeading());

        // Update the pose
        // if(AprilTagDetected()){
        //     double Xoffset = 0;//subject to change
        //     double Yoffset = 0;//subject to change
        //     double tempx = FieldD3Coords()[0];
        //     double tempy = FieldD3Coords()[1];
        //     double tempRot = FieldD3Coords()[2];
        //     odometry = new SwerveDriveOdometry(
        //         kinematics,
        //         navx.getGyroRotation2d(),
        //         getSwerveModulePositions(),
        //         new Pose2d(tempx+Xoffset, tempy+Yoffset, new Rotation2d(Math.PI))
        //     );
        // }else{

        // }

    }
    public void fieldSetPos(){
        field.setRobotPose(getFieldPos());
    }
    public void uppdateOdom(){
        // if(AprilTagDetected()){
        //     var gyroAngle = navx.getGyroRotation2d();
        //    double x = FieldD3Coords()[0];
        //    x+=8.24;
        //     double y = FieldD3Coords()[0];
        //     y+=4.05;
        //     odometry.resetPosition(
        //     navx.getGyroRotation2d(),
        //     getSwerveModulePositions(),
        //     new Pose2d(x, y, gyroAngle)
        // );
        // }else{
        odometry.update(getRotation(), getSwerveModulePositions());
        // }
    }




    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose2d) {
        odometry.resetPosition(
                navx.getGyroRotation2d(),
                getSwerveModulePositions(),
                pose2d
        );
    }




    public double getRetroXoffset(){
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(1);
        return table.getEntry("tx").getDouble(0);

    }
    public double getAprilXOffset(){
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(0);
        return table.getEntry("tx").getDouble(0);
    }

    public void setDriveCam(){
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(2);
    }

    public double getAprilID(){
        if(AprilTagDetected()){
            var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
            table.getEntry("pipeline").setNumber(0);
            return table.getEntry("tid").getDouble(-1);
        }
        return -1;
    }

    // public double[] FieldD3Coords(){
    //     if(AprilTagDetected()){
    //     var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
    //     table.getEntry("pipeline").setNumber(0);

    //     double vals[] = {table.getEntry("botpose").getDoubleArray(new double[21])[0],table.getEntry("botpose").getDoubleArray(new double[21])[1],table.getEntry("tid").getDouble(-1)};
    //     return vals;
    //     }
    //     return null;

    //     // return table.getEntry("botpose").getDoubleArray(new double[21]);
    // }


    public boolean AprilTagDetected(){
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(0);

        return table.getEntry("botpose").getDoubleArray(new double[21]).length==6;
    }
}