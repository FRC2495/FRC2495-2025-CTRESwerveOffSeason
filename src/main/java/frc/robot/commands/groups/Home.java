package frc.robot.commands.groups;

import java.util.List;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.auton.AutonConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.elevator.ElevatorMoveDownWithStallDetection;
import frc.robot.commands.neck.NeckMoveDownWithStallDetection;
import frc.robot.commands.algae_roller.*;
import frc.robot.commands.slider.*;


public class Home extends ParallelCommandGroup{
	
	public Home(Elevator elevator, Slider slider , Neck neck) {

		addCommands(

			new ElevatorMoveDownWithStallDetection(elevator),

			new SliderRetractWithLimitSwitch(slider),
			
			new NeckMoveDownWithStallDetection(neck)
		);
	}
}
