package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subystems.Drivetrain;

public class TankDriveCommand extends CommandBase {

  private final Drivetrain drivetrain;

  private final Joystick leftStick;
  private final Joystick rightStick;

  public TankDriveCommand(Drivetrain drivetrain, Joystick leftStick, Joystick rightStick) {
    this.drivetrain = drivetrain;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    addRequirements(this.drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.tankDrivePercent(leftStick.getY(), rightStick.getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
