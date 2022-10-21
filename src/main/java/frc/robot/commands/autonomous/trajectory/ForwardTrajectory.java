package frc.robot.commands.autonomous.trajectory;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subystems.Drivetrain;

public class ForwardTrajectory extends SequentialCommandGroup {

  private final SimpleMotorFeedforward motorFeed = new SimpleMotorFeedforward(Constants.AUTO_MOTOR_FEED_KS, Constants.AUTO_MOTOR_FEED_KV);
  private final PIDController PID = new PIDController(Constants.AUTO_PID_KP, Constants.AUTO_PID_KI, Constants.AUTO_PID_KD);
  private final RamseteController ramseteController = new RamseteController(Constants.AUTO_RAMSETE_CONFIG_B, Constants.AUTO_RAMSETE_CONFIG_ZETA);

  public ForwardTrajectory(Drivetrain drivetrain) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.AUTO_TRAJECTORY_MAX_VELMET, Constants.AUTO_TRAJECTORY_MAX_ACCMETSQ).setKinematics(drivetrain.getKinematics());
    Trajectory forward = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            // new Translation2d(1, 1),
            // new Translation2d(2, -1)
          ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
    drivetrain.resetOdometry(forward.getInitialPose());
    addCommands(
      new RamseteCommand(
        forward,
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
