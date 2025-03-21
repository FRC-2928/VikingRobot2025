// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.HumanPlayerPosition;
import frc.robot.Constants.ReefPosition;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterLimelight extends Command {
  /** Creates a new centerLimelight. */
  private Distance offsetX;
  private Distance offsetY;
  private Angle offsetTheta;
  private PIDController centerPIDx;
  private PIDController centerPIDy;
  private PIDController centerRotaionPid;
  private Pose2d tagPoseRobotspace;
  /// Tag Pose in Blue-Origin Space
  private Pose2d tagPose;
  private Pose2d transformedTagPose;
  /// Transform2d representing the transformation matrix for the tag
  private Transform2d tagPoseTransform;
  private List<Integer> tagsToCheck;

  private Distance xTolerance = Units.Inches.of(0.25);
  private Distance yTolerance = Units.Inches.of(0.5);
  private Angle thetaTolerance = Units.Degrees.of(0.5);

  private static final Distance offsetReef = Units.Inches.of(7/*Tuning.offsetCenterReef.get()*/);
    public final static List<Integer> reefTags = List.of(6,7,8,9,10,11,17,18,19,20,21,22);
      public CenterLimelight(Distance offsetX, Distance offsetY, final List<Integer> tagsToCheck) {
        this(offsetX, offsetY, Units.Radians.of(0), tagsToCheck);
      }

      /**
       * Center the robot on some tags... Treat the robot as a point for now and so X/Y are offsets to a single point (centerpoint) with some relative rotation Theta
       * Using tag-coordinates (+X to the right looking head-on, +Y down, +Z out of the page, 0* rot is in the direction of +Z & 180* rot is looking head-on)
       */
      public CenterLimelight(Distance offsetX, Distance offsetY, Angle offsetTheta, final List<Integer> tagsToCheck) {
        this.addRequirements(RobotContainer.getInstance().drivetrain);
        // X = forward/back distance away from the target in robot space, + half width since Robots are not points
        this.offsetX = offsetX.plus(Constants.Drivetrain.halfRobotWidthBumpersOn);
        this.offsetY = offsetY;
        // this.offsetTheta = offsetTheta;
        this.offsetTheta = offsetTheta.plus(Units.Radians.of(Math.PI));
        this.centerPIDx = Constants.Drivetrain.Auto.centerLimelight.createController();
        this.centerPIDy = Constants.Drivetrain.Auto.centerLimelight.createController();
        this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
        this.centerRotaionPid.enableContinuousInput(-Math.PI, Math.PI);
        this.tagsToCheck = tagsToCheck;
        this.tagPoseTransform = new Transform2d(this.offsetX, this.offsetY, new Rotation2d(this.offsetTheta));
        Logger.recordOutput("Drivetrain/CenterLimelight/xToleranceInches", xTolerance.in(Units.Inches));
        Logger.recordOutput("Drivetrain/CenterLimelight/yToleranceInches", yTolerance.in(Units.Inches));
        Logger.recordOutput("Drivetrain/CenterLimelight/thetaToleranceDegrees", thetaTolerance.in(Units.Degrees));
      }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      double closest = Double.MAX_VALUE;
      var curBotPoseTranslation = RobotContainer.getInstance().drivetrain.getEstimatedPosition().getTranslation();
      try {
        tagPose = Constants.FIELD_LAYOUT.getTagPose(17).get().toPose2d();
        for (int tag : tagsToCheck) {
          Pose2d curTagPose = Constants.FIELD_LAYOUT.getTagPose(tag).get().toPose2d();
          var distToTag = curBotPoseTranslation.getDistance(curTagPose.getTranslation());
          if(distToTag < closest){
              closest = distToTag;
              tagPose = curTagPose;
          }
        }
      } catch (Exception e) {
        // TODO: handle exception gracefully
      }
      Logger.recordOutput("Drivetrain/CenterLimelight/tagpose", tagPose);
    }

    @Override
    public void execute() {
      Pose2d robotPose = RobotContainer.getInstance().drivetrain.getEstimatedPosition();
      transformedTagPose = tagPose.transformBy(tagPoseTransform);  // this gets the centerpoint aligned properly; "move" the target
      Pose2d robotPoseTagSpace = robotPose.relativeTo(transformedTagPose);  // target is now our origin/reference pose

      // consider the robot to be a single point for now -- we've accounted for frame perimeter in the offset calcs
      double robotXTagSpace = robotPoseTagSpace.getX();
      double robotYTagSpace = robotPoseTagSpace.getY();
      Angle robotThetaTagSpace = robotPoseTagSpace.getRotation().getMeasure();

      // Check tolerances and zero any control efforts that are within the tolerance range
      var toleranceMetX = Units.Meters.of(robotPoseTagSpace.getX()).isNear(Units.Inches.of(0), xTolerance);
      var toleranceMetY = Units.Meters.of(robotPoseTagSpace.getY()).isNear(Units.Inches.of(0), yTolerance);
      var toleranceMetTheta = robotThetaTagSpace.isNear(Units.Radians.of(0), thetaTolerance);
      // robotXTagSpace = toleranceMetX ? 0 : robotXTagSpace;
      // robotYTagSpace = toleranceMetY ? 0 : robotYTagSpace;
      // robotThetaTagSpace = toleranceMetTheta ? Units.Radians.of(0) : robotThetaTagSpace;

      // Everything is relative to the desired centerpoint of the bot, so PIDs should seek 0 delta between poses
      double xSpeedPid = centerPIDx.calculate(toleranceMetX ? 0 : robotXTagSpace, 0);
      double ySpeedPid = centerPIDy.calculate(toleranceMetY ? 0 : robotYTagSpace, 0);
      double thetaSpeedPid = centerRotaionPid.calculate(toleranceMetTheta ? 0 : robotThetaTagSpace.in(Units.Radians), 0);

      // TODO: scale speeds up as necessary
      RobotContainer.getInstance().drivetrain
          .control(
                new ChassisSpeeds(
                  xSpeedPid,
                  ySpeedPid,
                  thetaSpeedPid
                )
      );

      Logger.recordOutput("Drivetrain/CenterLimelight/robotXTagSpace", robotXTagSpace);
      Logger.recordOutput("Drivetrain/CenterLimelight/robotYTagSpace",  robotYTagSpace);
      Logger.recordOutput("Drivetrain/CenterLimelight/robotThetaTagSpace",  robotThetaTagSpace.in(Units.Degrees));
      // Logger.recordOutput("Drivetrain/CenterLimelight/Center Is Finished", false);
      Logger.recordOutput("Drivetrain/CenterLimelight/XSpeedPid", xSpeedPid);
      Logger.recordOutput("Drivetrain/CenterLimelight/YSpeedPid", ySpeedPid);
      Logger.recordOutput("Drivetrain/CenterLimelight/ThetaSpeedPid", thetaSpeedPid);
      // Logger.recordOutput("Drivetrain/CenterLimelight/limelightHasValidTargets", RobotContainer.getInstance().drivetrain.limelight.hasValidTargets());
      // Logger.recordOutput("Drivetrain/CenterLimelight/Theta", RobotContainer.getInstance().drivetrain.limelight.getBotPose3d_TargetSpace().getRotation().getAngle());
      Logger.recordOutput("Drivetrain/CenterLimelight/robotPoseTagSpace", robotPoseTagSpace);
      Logger.recordOutput("Drivetrain/CenterLimelight/tagPoseRobotSpace", tagPoseRobotspace);
      Logger.recordOutput("Drivetrain/CenterLimelight/TransformedTagPose", transformedTagPose);
      Logger.recordOutput("Drivetrain/CenterLimelight/RobotGlobalPoseEstimate", robotPose);
      // Logger.recordOutput("Drivetrain/CenterLimelight/thetaSpeed", thetaSpeed);
      // Logger.recordOutput("Drivetrain/CenterLimelight/thetaPid", thetaPid);
      // Logger.recordOutput("Drivetrain/CenterLimelight/estRotation", RobotContainer.getInstance().drivetrain.getEstimatedPosition().getRotation());
      // Logger.recordOutput("Drivetrain/CenterLimelight/offsetX", offsetX);
      // Logger.recordOutput("Drivetrain/CenterLimelight/offsetTheta", offsetTheta.in(Units.Radians));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.getInstance().drivetrain.halt();
        // Logger.recordOutput("Drivetrain/CenterLimelight/Center Is Finished", true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      // 2 ways of looking at the end conditions:
      // 1. the pose is at/near the target pose
      // 2. the PID outputs are "close" to zero

      // First get the robot's current pose in TagSpace
      Pose2d robotPose = RobotContainer.getInstance().drivetrain.getEstimatedPosition();
      // Get the "new (0,0,0)" point by transforming the tag pose by the given offsets
      transformedTagPose = tagPose.transformBy(tagPoseTransform);  // this gets the centerpoint aligned properly

      // Compare to the target
      var robotPoseRelativeToGoal = robotPose.relativeTo(transformedTagPose);
      // check the values...
      // TODO: adjust thresholds
      var isXDone = robotPoseRelativeToGoal.getMeasureX().isNear(Units.Inches.of(0), xTolerance);
      // (Math.abs(robotPoseRelativeToGoal.getMeasureX().in(Units.Inches)) < 0.5);
      // var isYDone = (Math.abs(robotPoseRelativeToGoal.getMeasureY().in(Units.Inches)) < 0.5);
      var isYDone = robotPoseRelativeToGoal.getMeasureY().isNear(Units.Inches.of(0), yTolerance);
      // var isRotDone = (robotPoseRelativeToGoal.getRotation().getMeasure().isNear(Units.Degrees.of(0), Units.Degrees.of(5)));
      var isRotDone = (robotPoseRelativeToGoal.getRotation().getMeasure().isNear(Units.Degrees.of(0), thetaTolerance));
      Logger.recordOutput("Drivetrain/CenterLimelight/isXDone", isXDone);
      Logger.recordOutput("Drivetrain/CenterLimelight/isYDone", isYDone);
      Logger.recordOutput("Drivetrain/CenterLimelight/isRotDone", isRotDone);
      return (isXDone && isYDone && isRotDone);
      // return (Math.abs(xSpeedPid) < 0.09) && (Math.abs(ySpeedPid) < 0.2) && (Math.abs(thetaPid) < 0.15);
    }

  public static CenterLimelight centerLimelightLeft(){
      return new CenterLimelight(Units.Inches.of(3.85),offsetReef.negate(), reefTags);
  }
  public static CenterLimelight centerLimelightRight(){
    return new CenterLimelight(Units.Inches.of(3.85),offsetReef, reefTags);
  }

  public static CenterLimelight centerLimelightCenter(){
    return new CenterLimelight(Units.Inches.of(3.85),Units.Inches.of(0), reefTags);
  }

  public static CenterLimelight centerLimeLightPosition(ReefPosition reefPos) {
    return new CenterLimelight(Units.Inches.of(3.85), offsetReef.times(reefPos.getDirection())/*Units.Inches.of(6.5 * reefPos.getDirection())*/, reefPos.getTagID());
  }

  public static CenterLimelight centerLimelightHPReverse(HumanPlayerPosition hpPose) {
    return new CenterLimelight(Units.Inches.of(10), Units.Inches.of(-15), Units.Radians.of(Math.PI), hpPose.getTagID());
  }

  public static CenterLimelight centerLimelightClosestHP() {
    return new CenterLimelight(Units.Inches.of(3.85), Units.Inches.of(-8), Units.Radians.of(Math.PI), List.of(1, 2, 12, 13));
  }

  public static CenterLimelight centerLimelightProcessor() {
    return new CenterLimelight(Units.Inches.of(3.85), Units.Inches.of(-15), Units.Radians.of(0), List.of(3, 16));
  }
}
