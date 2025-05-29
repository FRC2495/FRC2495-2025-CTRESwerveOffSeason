package frc.robot.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.auton.AutonConstants;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.groups.ScoreFourthLevelCoralAndHome;
import frc.robot.subsystems.*;
import frc.robot.sensors.*;
//import frc.robot.auton.sp2.*;


public class StartingPositionTwoOneCoral extends SequentialCommandGroup {

	public StartingPositionTwoOneCoral(RobotContainer container, CommandSwerveDrivetrain drivetrain, CoralRoller coral_roller, Neck neck, Elevator elevator, Slider slider){

		addCommands(

			new DrivetrainSwerveRelative(drivetrain, container, createLeaveStartingZoneTrajectory(container)),

			new ScoreFourthLevelCoralAndHome(elevator, coral_roller, neck, slider)

		); 
  
	}

	
	public static Trajectory createLeaveStartingZoneTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the -X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these waypoints
			List.of(),
			// End straight ahead of where we started, facing forward
			new Pose2d(AutonConstants.DISTANCE_TO_SCORE_PRELOAD_CORAL, 0, Rotation2d.fromDegrees(0)),
			container.createAlmostMaxTrajectoryConfig());

		return trajectory;
	}


}