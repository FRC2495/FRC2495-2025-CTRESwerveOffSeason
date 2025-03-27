// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.interfaces.ICamera;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoAlignToReefTimed extends WaitCommand {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  //private Timer dontSeeTagTimer, stopTimer;
  private SwerveDrivetrain drivetrain;
  private ICamera apriltag_camera;
  private Joystick joystick;
  //private double tagID = -1;
  //private Pose2d targetPose;
  private static final double JOYSTICK_EXIT_THRESHOLD = 0.3;

  public AutoAlignToReefTimed(boolean isRightScore, SwerveDrivetrain drivetrain, ICamera apriltag_camera, Joystick joystick, double timeout) {
    super(timeout);
    xController = new PIDController(Constants.VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore; // left and right correspond to the poles when youre facing the reef head on (imagine you are the robot scoring, the left and right aligns with your left and right)
    this.drivetrain = drivetrain;
    this.apriltag_camera = apriltag_camera;
    this.joystick = joystick;
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
    System.out.println("AutoAlignToReefTimed: initialize");
    super.initialize();
    //tagID = apriltag_camera.getLatestID(); //we make the apriltag ID the one we are looking at the moment we press the button to auto align
  }

  @Override
  public void execute() {
    if (apriltag_camera.isTargetVisible()) { // if we see a target 
      
      // Side C with offset (4.903, 2.7)

      Transform3d currentTransform = apriltag_camera.getBestCameraToTargetTransform();

      //SmartDashboard.putNumber("x", apriltag_camera.getBestCameraToTargetX(currentTransform)); // lets us check in shuffleboard if the x is correct

      double xPower = MathUtil.clamp(xController.calculate(apriltag_camera.getBestCameraToTargetX(currentTransform), Constants.VisionConstants.X_LEFT_ALIGNMENT), -1, 1); //calculates power needed to get from current x position to desired x position
      //SmartDashboard.putNumber("xPower", xPower); // lets us check in shuffleboard if the x power is correct
      //System.out.println("The xPower for auto-align is: " + xPower);

      double yPower = MathUtil.clamp(yController.calculate(apriltag_camera.getBestCameraToTargetY(currentTransform), isRightScore ? Constants.VisionConstants.Y_RIGHT_ALIGNMENT : Constants.VisionConstants.Y_LEFT_ALIGNMENT), -1, 1);
      double rotPower = rotController.calculate(apriltag_camera.getBestCameraToTargetRotationRadians(currentTransform), Constants.VisionConstants.ROT_ALIGNMENT);
      //System.out.println("The yPower for auto-align is: " + yPower);
      //System.out.println("The rotPower for auto-align is: " + rotPower);

      drivetrain.driveRobotRelative(new ChassisSpeeds(-xPower, -yPower, rotPower));

    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoAlignToReefTimed: end");
    drivetrain.drive(0, 0, 0.0);
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    /*return this.dontSeeTagTimer.hasElapsed(Constants.VisionConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.VisionConstants.POSE_VALIDATION_TIME);*/

      Transform3d newTransform = apriltag_camera.getBestCameraToTargetTransform();
      return ((Math.abs(joystick.getX()) > JOYSTICK_EXIT_THRESHOLD) || (Math.abs(joystick.getY()) > JOYSTICK_EXIT_THRESHOLD)) 
      || ((Math.abs(apriltag_camera.getBestCameraToTargetX(newTransform) - (isRightScore ? Constants.VisionConstants.X_RIGHT_ALIGNMENT : Constants.VisionConstants.X_LEFT_ALIGNMENT)) < VisionConstants.X_ALIGNMENT_AUTO_TOLERANCE) 
      && (Math.abs(apriltag_camera.getBestCameraToTargetY(newTransform) - (isRightScore ? Constants.VisionConstants.Y_RIGHT_ALIGNMENT : Constants.VisionConstants.Y_LEFT_ALIGNMENT)) < VisionConstants.Y_ALIGNMENT_AUTO_TOLERANCE)
      && (Math.abs(apriltag_camera.getBestCameraToTargetRotationRadians(newTransform) - Constants.VisionConstants.ROT_ALIGNMENT) < Constants.VisionConstants.ROT_ALIGNMENT_AUTO_TOLERANCE)); 
  }
}