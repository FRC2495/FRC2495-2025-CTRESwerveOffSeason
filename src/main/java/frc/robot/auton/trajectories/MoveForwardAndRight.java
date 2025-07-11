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

// moves forward and right, ending oriented at specified heading
public class MoveForwardAndRight extends SequentialCommandGroup {

	private double forwardDistance;
	private double rightDistance;
	private double finalHeading;

	public MoveForwardAndRight(CommandSwerveDrivetrain drivetrain, RobotContainer container, double forwardDistance, double rightDistance, double finalHeading) {

		this.forwardDistance = forwardDistance;
		this.rightDistance = rightDistance;
		this.finalHeading = finalHeading;
		
		addCommands(
			new DrivetrainSwerveRelative(drivetrain, container, createForwardAndRightTrajectory(container))           
		); 

	}

	public Trajectory createForwardAndRightTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these waypoints
			List.of(),
			// End ahead of where we started, facing sideway
			// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
			new Pose2d(+forwardDistance, -rightDistance, Rotation2d.fromDegrees(/*-*/finalHeading)),
			container.createTrajectoryConfig());

		return trajectory;
	}
   
}