package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Controls;
import frc.robot.constants.DisplayInfo;
import frc.robot.constants.RobotInfo.HopperInfo;
import frc.robot.constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.led.LEDPattern;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ShootCommand extends Command {
  private final IShooterSubsystem shooter;
  private final IHopperSubsystem hopper;
  private final ShootingMode mode;
  private final LEDStrip ledStrip;
  private final boolean hopperReverse;

  private final boolean requireSeperateShootButton;

  private double startTime;

  public ShootCommand(
      IShooterSubsystem shooter,
      IHopperSubsystem hopper,
      LEDStrip ledStrip,
      ShootingMode mode,
      boolean hopperReverse,
      boolean requireSeperateShootButton) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.mode = mode;
    this.ledStrip = ledStrip;
    this.requireSeperateShootButton = requireSeperateShootButton;

    addRequirements(shooter, hopper, ledStrip);

    this.hopperReverse = hopperReverse;
  }

  public ShootCommand(
      IShooterSubsystem shooter,
      IHopperSubsystem hopper,
      LEDStrip ledStrip,
      ShootingMode mode,
      boolean requireSeperateShootButton) {
    this(shooter, hopper, ledStrip, mode, false, requireSeperateShootButton);
  }

  @Override
  public void initialize() {
    System.out.println("Starting AutoShoot!");
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    shooter.setShootingMode(mode);

    if (shooter.isReady()) {
      ledStrip.usePattern(DisplayInfo.readyPattern);

      if (!requireSeperateShootButton
          || RobotContainer.oi
              .operatorController()
              .getButton(Controls.OperatorControls.FeedShooterButton)
              .getAsBoolean()) {
        if (hopperReverse) {
          hopper.runBackwards(HopperInfo.HOPPER_SPEED);
          ;
        } else
          hopper.run(
              false,
              switch (mode) {
                case AMP -> 0.8;
                default -> HopperInfo.HOPPER_SPEED;
              });
      }
    } else {
      ledStrip.usePattern(DisplayInfo.notReadyPattern);
      if (Timer.getFPGATimestamp()
          > startTime
              + switch (mode) {
                case AMP -> 0.08;
                default -> 0;
              }) {
        hopper.stop();
      } else {
        hopper.run(false, HopperInfo.HOPPER_SPEED);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return hopper.getIR().isActive() && hopper.getIR().rawSensorIsClear();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShootingMode(ShootingMode.IDLE);
    hopper.stop();
    ledStrip.usePattern(LEDPattern.BLANK);
  }
}
