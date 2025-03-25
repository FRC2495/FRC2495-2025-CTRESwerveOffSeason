package frc.robot.interfaces;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ICamera extends Subsystem {

	public double getDistanceToTarget();
	
	public double getAngleToTurnToTarget();

	public Optional<EstimatedRobotPose> getGlobalPose();

	//public int getLatestID();

	public boolean isTargetVisible();

	public Transform3d getBestCameraToTargetPose();

	public double getBestCameraToTargetX(Transform3d currentPose);

	public double getBestCameraToTargetY(Transform3d currentPose);

	public double getBestCameraToTargetRotationRadians(Transform3d currentPose);
}
