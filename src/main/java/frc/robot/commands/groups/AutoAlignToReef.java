// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoAlignToReef extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveDrivetrain drivebase;
  private AprilTagCamera apriltag_camera;
  private double tagID = -1;

  public AutoAlignToReef(boolean isRightScore, SwerveDrivetrain drivebase, AprilTagCamera apriltag_camera) {
    xController = new PIDController(Constants.VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.apriltag_camera = apriltag_camera;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.VisionConstants.ROT_ALIGNMENT);
    rotController.setTolerance(Constants.VisionConstants.ROT_ALIGNMENT_TOLERANCE);

    xController.setSetpoint(Constants.VisionConstants.X_LEFT_ALIGNMENT);
    xController.setTolerance(Constants.VisionConstants.X_ALIGNMENT_TOLERANCE);

    yController.setSetpoint(isRightScore ? Constants.VisionConstants.Y_LEFT_ALIGNMENT : Constants.VisionConstants.Y_RIGHT_ALIGNMENT);
    yController.setTolerance(Constants.VisionConstants.Y_ALIGNMENT_TOLERANCE);

    tagID = apriltag_camera.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && apriltag_camera.getLatestID() == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(0, 0, false); //if we are at the correct setpoint position, we stop
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}