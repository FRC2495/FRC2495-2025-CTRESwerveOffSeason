// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AprilTags;
import frc.robot.RobotContainer;
import frc.robot.interfaces.ICamera;

/** Wrapper for PhotonCamera class */
public class AprilTagCamera extends PhotonCamera implements ICamera {

	//TODO: UPDATE CAM SETTINGS FOR NEW ROBOT
	private static final String DEFAULT_CAM_NAME = "AprilTagCam";
	private static final double CAMERA_X_METERS =  Units.inchesToMeters(+9.6); // x distance from the center of the robot
	private static final double CAMERA_Y_METERS =  Units.inchesToMeters(-4.1); // y distance offset from the center of the robot
	private static final double CAMERA_HEIGHT_METERS =  Units.inchesToMeters(8);
	private static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(8.75); // may need to change 
	private static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(10.0); // tilt of our camera (radians)
	private static final double CAMERA_ROLL_RADIANS = 0.0;
	private static final double CAMERA_YAW_RADIANS = Units.degreesToRadians(0);
	private List<PhotonPipelineResult> cachedResults = Collections.emptyList(); // we create an empty list so it's never null
	//private int latestID;

	private PhotonPoseEstimator estimator;
	private Transform3d robotToCam = new Transform3d(
		new Translation3d(CAMERA_X_METERS, CAMERA_Y_METERS, CAMERA_HEIGHT_METERS),
		new Rotation3d(CAMERA_ROLL_RADIANS, CAMERA_PITCH_RADIANS, CAMERA_YAW_RADIANS)
	);

	/**
     * Constructs a PhotonCamera using inherited superclass.
     * Instantiates the DEFAULT_CAM_NAME variable as the name of the camera. 
	 * This will allow the robot to connect the camera from the PhotonVision UI to this class.
	 * Instantiates the estimator as a PhotonPoseEstimator using the current year's field layout, 
	 * pose estimate setting and camera position on robot.
     */
	public AprilTagCamera() {
		super(DEFAULT_CAM_NAME);
		//estimator = new PhotonPoseEstimator(RobotContainer.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
	}

	/**
     * Updates the list of pipeline results sent by PhotonVision since the last call to getAllUnreadResults().
     * Stores all of the results into a List object to be called in other methods in this class.
	 * Called periodically in Robot.java.
     */
	public void updateCacheResults(){ // we call this periodically in robot.java 
		cachedResults = getAllUnreadResults(); // note: calling this function clears the internal FIFO queue. We call this exactly ONCE per loop.
		//var result = this.getLatestResult();
		/*if(result!=null && result.hasTargets()) {
			latestID = result.getBestTarget().getFiducialId();
		} else {
			latestID = -1;}
	*/}

	/**
     * Returns the estimated robot pose based on 3D tracking of AprilTags.
	 * Looks through the list of pipeline results sent by PhotonVision.
	 * Finds changes in the results that differ from current belief of position.
	 * Updates estimated robot pose based on changes of the AprilTags the robot sees.
     *
     * @return The estimated robot pose based on 3D tracking of ApriLTags
     */
	/*public Optional<EstimatedRobotPose> getGlobalPose(){
		Optional<EstimatedRobotPose> globalPose = Optional.empty();
		for (var change : cachedResults) {
			globalPose = estimator.update(change);
		}
		return globalPose;
	}*/

	/**
     * Returns the AprilTag target-to-camera transformation.
	 * Checks for targets in camera vision.
	 * If target is seen, returns the latest AprilTag target and sends (X, Y, Z, rotation coordinates), as shown in the UI.
     *
     * @return The AprilTag target-to-camera transformation
     */
	public Transform3d getBestCameraToTargetTransform(){
		Transform3d bestCameraToTarget = null;
		//var results = cachedResults;
		if (!cachedResults.isEmpty()) {
			// Camera processed a new frame since last
			// Get the last one in the list.
			var result = cachedResults.get(cachedResults.size() - 1);
			if (result.hasTargets()) {
				// At least one AprilTag was seen by the camera
				for (var target : result.getTargets()) {
					bestCameraToTarget = target.getBestCameraToTarget();
					//System.out.println("The pose result is: " + bestCameraToTarget);
				}
			}
			else
			{
				System.out.println("No targets found");
			}
		}
		return bestCameraToTarget;
	}

	/**
     * Returns the X coordinate from the latest target-to-camera transformation information.
	 * Checks for valid transformation information.
	 * Transforms the current AprilTag target and sends X coordinate, as seen in the UI.
     *
	 * @param currentPose The latest target-to-camera transformation information.
     * @return The X coordinate from the latest target-to-camera transformation information if found. If not, returns 0.0.
     */
	public double getBestCameraToTargetX(Transform3d currentPose) {
		if (currentPose != null) {
			return currentPose.getX(); 
		}
		System.out.println("No AprilTag target X coordinate found");
		return 0.0; 
	}

	/**
     * Returns the Y coordinate from the latest target-to-camera transformation information.
	 * Checks for valid transformation information.
	 * Transforms the current AprilTag target and sends Y coordinate, as seen in the UI.
     *
	 * @param currentPose The latest target-to-camera transformation information.
     * @return The Y coordinate from the latest target-to-camera transformation information if found. If not, returns 0.0.
     */
	public double getBestCameraToTargetY(Transform3d currentPose) {
		if (currentPose != null) {
			return currentPose.getY(); 
		}
		System.out.println("No AprilTag target Y coordinate found");
		return 0.0; 
	}

	/**
     * Returns the rotation of the robot based on  the latest target-to-camera transformation information.
	 * Checks for valid transformation information.
	 * Transforms the current AprilTag target and sends rotation data, as seen in the UI.
     *
	 * @param currentPose The latest target-to-camera transformation information.
     * @return The rotation of the robot compared to the AprilTag from the received target-to-camera transformation information in radians if found. If not, returns pi.
     */
	public double getBestCameraToTargetRotationRadians(Transform3d currentPose) {
		if (currentPose != null) {
			return currentPose.getRotation().getAngle();
		}
		System.out.println("No AprilTag target rotation coordinate found");
		return Math.PI; 
	}

	/**
     * Returns whether the AprilTag is in sight of the camera.
	 * Checks for target in camera view.
	 * Sends true if target is an AprilTag on the reef.
     *
     * @return Whether the AprilTag is in sight of the camera
     */
	public boolean isTargetVisible()
	{
		boolean targetVisible = false;
		//double targetYaw = 0.0;
		//var results = cachedResults;
		if (!cachedResults.isEmpty()) {
			// Camera processed a new frame since last
			// Get the last one in the list.
			var result = cachedResults.get(cachedResults.size() - 1);
			if (result.hasTargets()) {
				// At least one AprilTag was seen by the camera
				for (var target : result.getTargets()) {
					if (target.getFiducialId() == AprilTags.RED_REEF_SIDE_A 
					|| target.getFiducialId() == AprilTags.RED_REEF_SIDE_B 
					|| target.getFiducialId() == AprilTags.RED_REEF_SIDE_C 
					|| target.getFiducialId() == AprilTags.RED_REEF_SIDE_D 
					|| target.getFiducialId() == AprilTags.RED_REEF_SIDE_E 
					|| target.getFiducialId() == AprilTags.RED_REEF_SIDE_F 
					|| target.getFiducialId() == AprilTags.BLUE_REEF_SIDE_A 
					|| target.getFiducialId() == AprilTags.BLUE_REEF_SIDE_B 
					|| target.getFiducialId() == AprilTags.BLUE_REEF_SIDE_C 
					|| target.getFiducialId() == AprilTags.BLUE_REEF_SIDE_D 
					|| target.getFiducialId() == AprilTags.BLUE_REEF_SIDE_E 
					|| target.getFiducialId() == AprilTags.BLUE_REEF_SIDE_F) {
						// Found High Value Target, record its information
						//targetYaw = target.getYaw();
						targetVisible = true;
					}
				}
			}
		}
		return targetVisible;
	}
	
	/**
     * Returns distance in meters from camera to target.
	 * Calls getDistanceToHighValueTarget() method.
     *
     * @return The distance in meters from camera to target.
     */
	public double getDistanceToTarget() {
		//return getDistanceToBestTarget();
		return getDistanceToHighValueTarget();
	}

	/**
     * Returns the angle in radians from camera to middle of target.
	 * Calls getAngleToTurnToHighValueTarget() method.
     *
     * @return The angle in radians from camera to middle of target.
     */
	public double getAngleToTurnToTarget() {
		//double angle = getAngleToTurnToBestTarget();
		double angle = getAngleToTurnToHighValueTarget();

		/*if (angle != 0.0) // only if we can see a target, so we continue to return 0.0 if we don't see one.
		{
			angle += APRILTAG_CAMERA_SHOOTER_ALIGNMENT_CORRECTION_DEGREES; // apply offset in degrees to compensate for shooter being a bit crooked
		}*/

		return angle;
	}

	/**
     * Returns the distance from the AprilTag to the camera in inches only if the target is considered a best target.
	 * Checks if the latest AprilTag target received is the best target in list before retrieving.
     *
	 * @throws Exception If no AprilTag target is found or target found is not considered a best target
     * @return The distance from the AprilTag to the camera in inches only if the target is considered a best target.
     */

	public double getDistanceToBestTarget() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			if (result.hasTargets() && result.getBestTarget()!=null) {
				double range = PhotonUtils.calculateDistanceToTargetMeters(
					CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
					Units.degreesToRadians(result.getBestTarget().getPitch())
					);
				return Units.metersToInches(range);
			}
			return 0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}
	}

	/**
     * Returns yaw of the target compared to the camera if the target is considered a best target.
	 * Checks if the latest AprilTag target received is the best target in list before retrievin.
     *
	 * @throws Exception If no AprilTag target was found or was not the best target if found.
     * @return The yaw of the target compared to the camera if the target is considered a best target.
     */
	public double getYaw() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			/* The yaw of the target in degrees (positive right). */
			return result.hasTargets() && result.getBestTarget()!=null? 
				result.getBestTarget().getYaw():
				0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}	
	}

	/**
     * Returns yaw of the target compared to the camera if the target is considered a best target.
	 * Calls getYaw() method and returns as positive.
     *
     * @return The positive yaw of the target compared to the camera if the target is considered a best target.
     */
	public double getAngleToTurnToBestTarget()
	{
		return +getYaw();
	}

	/*public int getLatestID(){
		return latestID;
	}*/

	/**
     * Returns pitch of the target compared to the camera if the target is a best target.
	 * Checks if the latest AprilTag target received is the best target in list before retrieving.
	 * 
	 * @throws Exception If no AprilTag target found or target found is not considered a best target.
     * @return The pitch of the target compared to the camera if the target is considered a best target.
     */
	public double getPitch() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			/* The pitch of the target in degrees (positive up). */
			return result.hasTargets() && result.getBestTarget()!=null? 
				result.getBestTarget().getPitch():
				0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}
	
	}

	/**
     * Returns skew of the target compared to the camera if the target is a best target.
	 * Checks if the latest AprilTag target received is the best target in list before retrieving.
	 * 
	 * @throws Exception If no AprilTag target found or target found is not considered a best target.
     * @return The skew of the target compared to the camera if the target is considered a best target.
     */
	public double getSkew() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			/* The skew of the target in degrees (counter-clockwise positive). */
			return result.hasTargets() && result.getBestTarget()!=null? 
				result.getBestTarget().getSkew():
				0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}	
	}

	/**
     * Returns AprilTag ID in camera vision if the target is a best target.
	 * Checks if the latest AprilTag target received is the best target in list before retrieving.
	 * 
	 * @throws Exception If no AprilTag target found or target found is not considered a best target.
     * @return The AprilTag ID in view of camera if it is considered a best target.
     */
	public int getAprilTagId() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			return result.hasTargets() && result.getBestTarget()!=null? 
				result.getBestTarget().getFiducialId():
				0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0;
		}
	}

	/**
     * Returns AprilTag ID in camera vision if the target is a high value target.
	 * Checks if the latest AprilTag target received is a high value target.
	 * In this case, a high value target is a target on the reef.
	 * 
	 * @param result The latest result from the camera pipeline.
     * @return The AprilTag ID in view of camera if it is considered a best target.
     */
	public PhotonTrackedTarget getHighValueTarget(PhotonPipelineResult result) {

		if (result.hasTargets()) {

			List<PhotonTrackedTarget> targets = result.getTargets();

			for (PhotonTrackedTarget target: targets) 
			{
				int targetId = target.getFiducialId();

				if (targetId == AprilTags.RED_REEF_SIDE_A 
					|| targetId == AprilTags.RED_REEF_SIDE_B 
					|| targetId == AprilTags.RED_REEF_SIDE_C 
					||  targetId == AprilTags.RED_REEF_SIDE_D 
					|| targetId == AprilTags.RED_REEF_SIDE_E 
					|| targetId == AprilTags.RED_REEF_SIDE_F 
					|| targetId == AprilTags.BLUE_REEF_SIDE_A 
					|| targetId == AprilTags.BLUE_REEF_SIDE_B 
					|| targetId == AprilTags.BLUE_REEF_SIDE_C 
					|| targetId == AprilTags.BLUE_REEF_SIDE_D 
					|| targetId == AprilTags.BLUE_REEF_SIDE_E 
					|| targetId == AprilTags.BLUE_REEF_SIDE_F)
				{
					return target; // SUPER high value target found - more important than high value only
				}
			}

			/*for (PhotonTrackedTarget target: targets) 
			{
				int targetId = target.getFiducialId();

				if (targetId == AprilTags.RED_AMP
					|| targetId == AprilTags.BLUE_AMP)
				{
					return target; // high value target found
				}
			}*/
		}

		return null; // no high value target found
	}

	/**
     * Returns distance from AprilTag target to camera in inches if the target is a high value target.
	 * Checks if the latest AprilTag target received is a high value target before retrieving.
	 * 
	 * @throws Exception If no AprilTag target found or target found is not a high value target
     * @return The AprilTag ID in view of camera if it is considered a best target.
     */
	public double getDistanceToHighValueTarget() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			if (result.hasTargets() && getHighValueTarget(result)!=null) {
				double range = PhotonUtils.calculateDistanceToTargetMeters(
					CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
					Units.degreesToRadians(getHighValueTarget(result).getPitch())
					);
				return Units.metersToInches(range);
			}
			return 0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}
	}

	/**
     * Returns angle between the center of the high value target and the camera in radians.
	 * Checks if the latest AprilTag target is a high value target before recieving.
	 * 
	 * @throws Exception If the AprilTag target is a high value target and is valid.
     * @return The angle between the center of the high value target and the camera in radians.
     */
	public double getAngleToTurnToHighValueTarget()
	{
		try
		{
			PhotonPipelineResult result = getLatestResult();

			/* The yaw of the target in degrees (positive right). */
			return result.hasTargets() && getHighValueTarget(result)!=null? 
				getHighValueTarget(result).getYaw():
				0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}
	}
}

