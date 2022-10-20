package frc.robot.commands.autonomous.trajectory;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subystems.Drivetrain;

public class STrajectory extends SequentialCommandGroup {

  private final SimpleMotorFeedforward motorFeed = new SimpleMotorFeedforward(Constants.AUTO_MOTOR_FEED_KS, Constants.AUTO_MOTOR_FEED_KV);
  private final PIDController PID = new PIDController(Constants.AUTO_PID_KP, Constants.AUTO_PID_KI, Constants.AUTO_PID_KD);
  private final RamseteController ramseteController = new RamseteController(Constants.AUTO_RAMSETE_CONFIG_B, Constants.AUTO_RAMSETE_CONFIG_ZETA);

  public STrajectory(Drivetrain drivetrain) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.AUTO_TRAJECTORY_MAX_VELMET, Constants.AUTO_TRAJECTORY_MAX_ACCMETSQ).setKinematics(drivetrain.getKinematics());

    Trajectory S_path = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    drivetrain.resetOdometry(S_path.getInitialPose());
    addCommands(
        /**
         * Constructs a new RamseteCommand that, when executed, will follow the provided
         * trajectory. PID
         * control and feedforward are handled internally, and outputs are scaled -12 to
         * 12 representing
         * units of volts.
         *
         * <p>
         * Note: The controller will *not* set the outputVolts to zero upon completion
         * of the path -
         * this is left to the user, since it is not appropriate for paths with
         * nonstationary endstates.
         *
         * @param trajectory      The trajectory to follow.
         * @param pose            A function that supplies the robot pose - use one of
         *                        the odometry classes to
         *                        provide this.
         * @param controller      The RAMSETE controller used to follow the trajectory.
         * @param feedforward     The feedforward to use for the drive.
         * @param kinematics      The kinematics for the robot drivetrain.
         * @param wheelSpeeds     A function that supplies the speeds of the left and
         *                        right sides of the robot
         *                        drive.
         * @param leftController  The PIDController for the left side of the robot
         *                        drive.
         * @param rightController The PIDController for the right side of the robot
         *                        drive.
         * @param outputVolts     A function that consumes the computed left and right
         *                        outputs (in volts) for
         *                        the robot drive.
         * @param requirements    The subsystems to require.
         */
        new RamseteCommand(
            S_path,
            drivetrain::getPose,
            ramseteController,
            motorFeed,
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            PID, PID,
            drivetrain::tankDriveVolts,
            drivetrain
        )
      );
  }
}
