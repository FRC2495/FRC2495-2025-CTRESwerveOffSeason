package frc.robot.auton.trajectories;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;

// moves forward and right at 90 degrees
public class MoveForwardAndHardRight extends SequentialCommandGroup {

	private double forwardDistance;
	private double rightDistance;

	public MoveForwardAndHardRight(CommandSwerveDrivetrain drivetrain, RobotContainer container, double forwardDistance, double rightDistance) {

		this.forwardDistance = forwardDistance;
		this.rightDistance = rightDistance;
		
		addCommands(
			new DrivetrainSwerveRelative(drivetrain, container, createForwardAndHardRightTrajectory(container))           
		); 

	}

	public Trajectory createForwardAndHardRightTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these waypoints
			List.of(),
			// End ahead of where we started, facing sideway
			// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
			new Pose2d(+forwardDistance, -rightDistance, Rotation2d.fromDegrees(-90)),
			container.createTrajectoryConfig());

		return trajectory;
	}
   
}