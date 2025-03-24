// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoAlignToReef extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveDrivetrain drivetrain;
  private AprilTagCamera apriltag_camera;
  private double tagID = -1;
  private Pose2d targetPose;

  public AutoAlignToReef(boolean isRightScore, SwerveDrivetrain drivetrain, AprilTagCamera apriltag_camera) {
    xController = new PIDController(Constants.VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivetrain = drivetrain;
    this.apriltag_camera = apriltag_camera;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    /*this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start(); //might not need the timers*/

    /*rotController.setSetpoint(Constants.VisionConstants.ROT_ALIGNMENT);
    rotController.setTolerance(Constants.VisionConstants.ROT_ALIGNMENT_TOLERANCE);

    xController.setSetpoint(Constants.VisionConstants.X_LEFT_ALIGNMENT); // x offset when we score on the left reef pole (same for left and right)
    xController.setTolerance(Constants.VisionConstants.X_ALIGNMENT_TOLERANCE); // how much offset we allow 

    yController.setSetpoint(isRightScore ? Constants.VisionConstants.Y_LEFT_ALIGNMENT : Constants.VisionConstants.Y_RIGHT_ALIGNMENT); 
    yController.setTolerance(Constants.VisionConstants.Y_ALIGNMENT_TOLERANCE); 
*/
    tagID = apriltag_camera.getLatestID(); //we make the apriltag ID the one we are looking at the moment we press the button to auto align
  }

  @Override
  public void execute() {
    if (apriltag_camera.isTargetVisible() && apriltag_camera.getLatestID() == tagID) { // if we see a target + the target is the one we are aiming for then..
      this.dontSeeTagTimer.reset();
      
      double[] positions = new double[6];
      positions[2] = apriltag_camera.getBestCameraToTargetX(); // i have no idea if this method is doing what i think it is doing lol
      positions[0] = apriltag_camera.getBestCameraToTargetY();
      positions[4] = apriltag_camera.getBestCameraToTargetRotationRadians();
      SmartDashboard.putNumber("x", positions[2]); // lets us check in shuffleboard if the x is correct

      double xPower = MathUtil.clamp(xController.calculate(positions[2], Constants.VisionConstants.X_LEFT_ALIGNMENT), -1, 1); //calculates power needed to get from current x position to desired x position
      SmartDashboard.putNumber("xPower", xPower); // lets us check in shuffleboard if the x power is correct

      double yPower = MathUtil.clamp(yController.calculate(positions[0], isRightScore ? Constants.VisionConstants.Y_RIGHT_ALIGNMENT : Constants.VisionConstants.Y_LEFT_ALIGNMENT), -1, 1);
      double rotPower = rotController.calculate(positions[4], Constants.VisionConstants.ROT_ALIGNMENT);

      //drivetrain.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
      drivetrain.drive(new ChassisSpeeds(-yPower, xPower, rotPower)); //not sure if the negative is needed


      //   if (!rotController.atSetpoint() ||
      //       !yController.atSetpoint() ||
      //       !xController.atSetpoint()) {
      //     stopTimer.reset();
      //   }
      // } else {
      //   drivetrain.drive(0.0, 0.0, 0.0); //if we are at the correct setpoint position, we stop
      // }

      //SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0.0);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    /*return this.dontSeeTagTimer.hasElapsed(Constants.VisionConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.VisionConstants.POSE_VALIDATION_TIME);*/
      return (Math.abs(apriltag_camera.getBestCameraToTargetX()-(isRightScore ? Constants.VisionConstants.X_RIGHT_ALIGNMENT : Constants.VisionConstants.X_LEFT_ALIGNMENT))<VisionConstants.X_ALIGNMENT_TOLERANCE) 
      && (Math.abs(apriltag_camera.getBestCameraToTargetY()-(isRightScore ? Constants.VisionConstants.Y_RIGHT_ALIGNMENT : Constants.VisionConstants.Y_LEFT_ALIGNMENT))<VisionConstants.Y_ALIGNMENT_TOLERANCE)
      && (Math.abs(apriltag_camera.getBestCameraToTargetRotationRadians()-Constants.VisionConstants.ROT_ALIGNMENT)<VisionConstants.ROT_ALIGNMENT_TOLERANCE); 
  }
}