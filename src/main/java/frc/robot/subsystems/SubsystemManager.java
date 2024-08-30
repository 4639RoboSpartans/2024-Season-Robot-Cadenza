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
import frc.robot.subsystems.hopper.DummyHopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.DummyIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.DummyShooterSubsystem;
import frc.robot.subsystems.shooter.FalconShooterSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooter.pivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooter.pivot.IShooterPivotSubsystem;
import frc.robot.subsystems.shooter.pivot.NeoShooterPivotSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

import static frc.robot.constants.Constants.currentRobot;

public class SubsystemManager {
    private static NavX navX;

    private static LEDStrip ledStrip;

    private static ISwerveDriveSubsystem swerveDrive;

    private static IShooterSubsystem shooter;
    private static IShooterPivotSubsystem shooterPivot;
    private static IIntakeSubsystem intake;
    private static IHopperSubsystem hopper;
    private static IClimberSubsystem climber;

    private static OI oi;

    public static NavX getNavX() {
        if(navX == null) {
            navX = new NavX();
        }
        return navX;
    }

    public static OI getOI() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }

    public static LEDStrip getLedStrip() {
        if(ledStrip == null) {
            ledStrip = switch(currentRobot){
                case ZEUS -> new DummyLEDStrip();
                case SIREN -> new PhysicalLEDStrip(0, 64);
            };
        }
        return ledStrip;
    }

    public static ISwerveDriveSubsystem getSwerveDrive() {
        if(swerveDrive == null) {
            swerveDrive = TunerConstants.DriveTrain;
        }
        return swerveDrive;
    }

    public static IShooterPivotSubsystem getShooterPivot(IShooterSubsystem shooter) {
        if(shooterPivot == null) {
            shooterPivot = switch(currentRobot){
                case ZEUS -> new DummyShooterPivotSubsystem();
                case SIREN -> new NeoShooterPivotSubsystem(IDs.SHOOTER_PIVOT_MOTOR_LEFT, IDs.SHOOTER_PIVOT_MOTOR_RIGHT, shooter);
            };
        }
        return shooterPivot;
    }

    public static IShooterSubsystem getShooter() {
        if(shooter == null) {
            shooter = switch (currentRobot) {
                case ZEUS -> new DummyShooterSubsystem();
                case SIREN -> new FalconShooterSubsystem(
                    IDs.SHOOTER_SHOOTER_LEFT_MOTOR,
                    IDs.SHOOTER_SHOOTER_RIGHT_MOTOR
                );
            };
        }
        return shooter;
    }

    public static IIntakeSubsystem getIntake() {
        if(intake == null) {
            intake = switch(currentRobot){
                case ZEUS -> new DummyIntakeSubsystem();
                case SIREN -> //new DummyIntakeSubsystem();
                 new IntakeSubsystem(
                    IDs.INTAKE_PIVOT_MOTOR_LEFT,
                    IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                    IDs.INTAKE_MOTOR,
                    IDs.INTAKE_ENCODER_DIO_PORT
                );
                // case SIREN -> new DummyIntakeSubsystem();
            };
        };
        return intake;
    }

    public static IHopperSubsystem getHopper() {
        if(hopper == null) {
            hopper = switch(currentRobot){
                case ZEUS -> new DummyHopperSubsystem();
                case SIREN -> //new DummyHopperSubsystem();
                 new HopperSubsystem(IDs.HOPPER_MOTOR);
            };
        }
        return hopper;
    }

    public static IClimberSubsystem getClimber() {
        if(climber == null) {
            climber = switch(currentRobot){
                case ZEUS -> new DummyClimberSubsystem();
                case SIREN -> new ClimberSubsystem(IDs.CLIMBER_LEFT, IDs.CLIMBER_RIGHT);
            };
        }
        return climber;
    }
}
