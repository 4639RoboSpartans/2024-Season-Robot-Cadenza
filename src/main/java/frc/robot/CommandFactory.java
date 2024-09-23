package frc.robot;

import frc.lib.CSM.CommandStateMachine;
import frc.lib.CSM.State;
import frc.robot.constants.Controls;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.pivot.IPivot;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.subsystems.shooter.shooter.IShooter;
import frc.robot.subsystems.shooter.shooter.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class CommandFactory {
    private static final IShooter shooter = Shooter.getInstance();
    private static final IPivot pivot = Pivot.getInstance();
    private static IHopperSubsystem hopper = Hopper.getInstance();
    private static IIntakeSubsystem intake = Intake.getInstance();
    private static ISwerveDriveSubsystem swerve = CommandSwerveDrivetrain.getInstance();
    public class Teleop {
        public static final CommandStateMachine superstructureStateMachine = new CommandStateMachine();
        static {
            //create all states
            State idleState = superstructureStateMachine.addState(shooter.idle(), pivot.idle(), hopper.stop(), intake.pivotRetract());
            State hasNoteState = superstructureStateMachine.addState(shooter.idle(), pivot.idle(), hopper.stop(), intake.pivotRetract());
            State SOTFState = superstructureStateMachine.addState(shooter.autoShoot(), pivot.autoShoot(), hopper.feed(), intake.pivotRetract());
            State spinupState = superstructureStateMachine.addState(shooter.autoShoot(), pivot.autoShoot(), hopper.stop(), intake.pivotRetract());
            State manualState = superstructureStateMachine.addState(shooter.manualShoot(), pivot.manualShoot(), hopper.feed(), intake.pivotRetract());
            State launchSpinupState = superstructureStateMachine.addState(shooter.launch(), pivot.launch(), hopper.stop(), intake.pivotRetract());
            State launchState = superstructureStateMachine.addState(shooter.launch(), pivot.launch(), hopper.feed(), intake.pivotRetract());
            State intakeState = superstructureStateMachine.addState(shooter.idle(), pivot.idle(), hopper.feed(), intake.pivotIntake());
            State outtakeState = superstructureStateMachine.addState(shooter.idle(), pivot.idle(), hopper.outtake(), intake.pivotOuttake());
            State ampPrep = superstructureStateMachine.addState(shooter.idle(), pivot.idle(), hopper.outtake(), intake.ampPrep());
            State ampReady = superstructureStateMachine.addState(shooter.idle(), pivot.idle(), hopper.stop(), intake.pivotRetract());
            State amping = superstructureStateMachine.addState(shooter.idle(), pivot.idle(), hopper.stop(), intake.pivotAmp());

            //define all transitions
            idleState.on(Controls.OperatorControls.IntakeButton, intakeState); // complete
            intakeState.on(Controls.OperatorControls.IntakeButton.negate(), idleState);

            intakeState.on(hopper.hasNote(), hasNoteState);

            hasNoteState.on(Controls.OperatorControls.OuttakeButton, outtakeState);
            outtakeState.on(Controls.OperatorControls.OuttakeButton.negate(), hasNoteState);

            outtakeState.on(hopper.hasNote().negate(), idleState);

            hasNoteState.on(swerve.inShootingRange().and(swerve.inShootingSector()).and(Controls.DriverControls.SOTF), spinupState);
            spinupState.on(Controls.DriverControls.SOTF.negate(), hasNoteState);

            spinupState.on(swerve.isAligned(), SOTFState);
            SOTFState.on(swerve.isAligned().negate(), spinupState);
            SOTFState.on(Controls.DriverControls.SOTF.negate(), hasNoteState);




            superstructureStateMachine.setInitial(idleState);
        }
    }
}
