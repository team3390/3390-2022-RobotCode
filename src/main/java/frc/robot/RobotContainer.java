/**
 * @author CanOkan, DenizOzturk, DenizOndes
 * @since 12-09-2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.LockTargetCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.subystems.BallSubsystem;
import frc.robot.subystems.Drivetrain;

public class RobotContainer {

    /**
     * Subsystem objelerinin tanımlanması
     */
    private final Drivetrain drivetrain = new Drivetrain();
    private final BallSubsystem ballSubsystem = new BallSubsystem();

    /**
     * Joystick objelerinin tanımlanması
     */
    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final Joystick atari = new Joystick(3);

    public RobotContainer() {
        // Ana komutun TankDriveCommand olarak belirlenmesi
        drivetrain.setDefaultCommand(new TankDriveCommand(drivetrain, leftStick.getY(), rightStick.getY()));

        // Joystick butonlarına işlev atamaları yapılması
        new JoystickButton(atari, 4).whenHeld(new LockTargetCommand(drivetrain));
    }
  
    /**
     * Yarışmanın otonom periodunda çalışacak olan komutun
     * döndürülmesini sağlayan bir fonksiyon.
     * 
     * @return Robot sürücüleri tarafından seçilen otonom kodu.
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
