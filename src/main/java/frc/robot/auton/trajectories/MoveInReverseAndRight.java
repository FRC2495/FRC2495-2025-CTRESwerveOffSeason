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

// moves in reverse and right, ending oriented at specified heading
public class MoveInReverseAndRight extends SequentialCommandGroup {

	private double reverseDistance;
	private double rightDistance;
	private double finalHeading;

	public MoveInReverseAndRight(CommandSwerveDrivetrain drivetrain, RobotContainer container,  double reverseDistance, double rightDistance, double finalHeading) {

		this.reverseDistance = reverseDistance;
		this.rightDistance = rightDistance;
		this.finalHeading = finalHeading;

		addCommands(

			new DrivetrainSwerveRelative(drivetrain, container, createReverseAndRightTrajectory(container))
		   
		); 
  
	}

	public Trajectory createReverseAndRightTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the -X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(180.0)),
			// Pass through these waypoints
			List.of(),
			// End ahead of where we started, facing sideway
			// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
			new Pose2d(+reverseDistance, +rightDistance, Rotation2d.fromDegrees(/*-*/finalHeading)),
			container.createReverseTrajectoryConfig());

		return trajectory;
	}

   
}