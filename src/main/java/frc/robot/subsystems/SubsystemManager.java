package frc.robot.subsystems;

import frc.robot.constants.IDs;
import frc.robot.generated.TunerConstants;
import frc.robot.led.DummyLEDStrip;
import frc.robot.led.PhysicalLEDStrip;
import frc.robot.led.LEDStrip;
import frc.robot.oi.OI;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.DummyClimberSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.hopper.SimHopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.SimIntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.ConcreteIntakeSubsystem;
import frc.robot.subsystems.shooter.DummyShooterSubsystem;
import frc.robot.subsystems.shooter.FalconShooterFeedforward;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooter.pivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooter.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.pivot.NeoShooterPivotSubsystem;
import frc.robot.subsystems.swerve.DummySwerveDriveSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

import static frc.robot.constants.Constants.currentRobot;

public class SubsystemManager {
    private static OI oi;
    private static LEDStrip ledStrip;
    private static ISwerveDriveSubsystem swerveDrive;
    private static IShooterSubsystem shooter;
    private static PivotSubsystem shooterPivot;
    private static IntakeSubsystem intake;
    private static HopperSubsystem hopper;
    private static IClimberSubsystem climber;

    public static OI getOI() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }

    public static LEDStrip getLedStrip() {
        if(ledStrip == null) {
            ledStrip = switch(currentRobot){
                case ZEUS, SIM -> new DummyLEDStrip();
                case CADENZA -> new PhysicalLEDStrip(0, 64);
            };
        }
        return ledStrip;
    }

    public static ISwerveDriveSubsystem getSwerveDrive() {
        if(swerveDrive == null) {
            swerveDrive = switch (currentRobot) {
                case ZEUS -> new DummySwerveDriveSubsystem();
                case CADENZA, SIM -> TunerConstants.DriveTrain;
            };
        }
        return swerveDrive;
    }

    public static PivotSubsystem getShooterPivot(IShooterSubsystem shooter) {
        if(shooterPivot == null) {
            shooterPivot = switch(currentRobot){
                case ZEUS -> new DummyShooterPivotSubsystem();
                case CADENZA, SIM -> new NeoShooterPivotSubsystem(IDs.SHOOTER_PIVOT_MOTOR_LEFT, IDs.SHOOTER_PIVOT_MOTOR_RIGHT, shooter);
            };
        }
        return shooterPivot;
    }

    public static IShooterSubsystem getShooter() {
        if(shooter == null) {
            shooter = switch (currentRobot) {
                case ZEUS -> new DummyShooterSubsystem();
                case CADENZA, SIM -> new FalconShooterFeedforward(
                    IDs.SHOOTER_SHOOTER_LEFT_MOTOR,
                    IDs.SHOOTER_SHOOTER_RIGHT_MOTOR
                );
            };
        }
        return shooter;
    }

    public static IntakeSubsystem getIntake() {
        if(intake == null) {
            intake = switch(currentRobot){
                case ZEUS, SIM -> new SimIntakeSubsystem();
                case CADENZA -> new ConcreteIntakeSubsystem();
            };
        };
        return intake;
    }

    public static HopperSubsystem getHopper() {
        if(hopper == null) {
            hopper = switch(currentRobot){
                case ZEUS, SIM -> new SimHopperSubsystem();
                case CADENZA -> new HopperSubsystem(IDs.HOPPER_MOTOR);
            };
        }
        return hopper;
    }

    public static IClimberSubsystem getClimber() {
        if(climber == null) {
            climber = switch(currentRobot){
                case ZEUS, SIM -> new DummyClimberSubsystem();
                case CADENZA -> new ClimberSubsystem(IDs.CLIMBER_LEFT, IDs.CLIMBER_RIGHT);
            };
        }
        return climber;
    }
}
