package frc.robot.auton.trajectories;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.auton.AutonConstants;
import frc.robot.subsystems.*;

// leaves the community
public class CompletelyLeaveCommunity extends SequentialCommandGroup {

	public CompletelyLeaveCommunity(CommandSwerveDrivetrain drivetrain, RobotContainer container) {

		addCommands(
			new MoveInReverse(drivetrain, container, AutonConstants.DISTANCE_FROM_STARTING_POSITION_TO_LEAVE_COMMUNITY_METERS)            
		); 
  
	}
   
}