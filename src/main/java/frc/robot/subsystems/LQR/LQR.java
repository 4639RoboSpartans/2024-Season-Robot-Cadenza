package frc.robot.subsystems.LQR;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LQR extends SubsystemBase {
    private final LinearSystem<N1, N1, N1> swervePlant;
    private final KalmanFilter<N1, N1, N1> observer;
    private final LinearQuadraticRegulator<N1, N1, N1> controller;
    private final LinearSystemLoop<N1, N1, N1> loop;
    private double setPoint;
    public LQR(double kV,
               double kA,
               double MOI,
               double gearing,
               double modelAccuracy,
               double encoderAccuracy,
               double kQ,
               double kR,
               double frameTime,
               double maxVoltage){
        swervePlant = LinearSystemId.identifyVelocitySystem(kV, kA);
        observer = new KalmanFilter<>(
                Nat.N1(),
                Nat.N1(),
                swervePlant,
                VecBuilder.fill(modelAccuracy),
                VecBuilder.fill(encoderAccuracy),
                frameTime);
        controller = new LinearQuadraticRegulator<>(
                        swervePlant,
                        VecBuilder.fill(kQ),
                        VecBuilder.fill(kR),
                        frameTime);
        loop = new LinearSystemLoop<>(swervePlant, controller, observer, maxVoltage, frameTime);

    }

    @Override
    public void periodic(){

    }
}
