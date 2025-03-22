package frc.robot.interfaces;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ICamera extends Subsystem {

	public double getDistanceToTarget();
	
	public double getAngleToTurnToTarget();

	public Optional<EstimatedRobotPose> getGlobalPose();

	public int getLatestID();
}
