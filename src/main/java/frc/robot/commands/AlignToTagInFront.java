package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.PhotonCameraWrapper;

import java.util.function.Supplier;

import org.ejml.dense.row.misc.TransposeAlgs_CDRM;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignToTagInFront extends CommandBase {

    //???
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Omega_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    //Tag we want to align to
    private static int alignTag = VisionConstants.kAlignTag;

    //where we want to be in relation to the tag
    private static final Transform3d PositionToTag = new Transform3d(
        new Translation3d(VisionConstants.kInFrontTagDistance, 0, 0),
        new Rotation3d(0, 0.0, Math.PI));

    //private instance variables
    //private final PhotonCamera photonCamera = new PhotonCamera(VisionConstants.cameraName);
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kCamName);
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Pose2d> poseProvider;

    //PID controllers for the 3 axees
    //not sure if P,I and D are correct, we stole it from a video
    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, Omega_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    //Constructor
    //public AlignToTag(PhotonCamera cam, DriveSubsystem drive, Supplier<Pose2d> pose) {
    //    camera = cam;
    public AlignToTagInFront(DriveSubsystem drive, Supplier<Pose2d> pose) {
        driveSubsystem = drive;
        poseProvider = pose;

        xController.setTolerance(VisionConstants.kXVariability);
        yController.setTolerance(VisionConstants.kYVariability);
        omegaController.setTolerance(Units.degreesToRadians(VisionConstants.kOmegaVariability));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveSubsystem);
    }

    //Resets everything on startup
    @Override
    public void initialize() {
        lastTarget = null;
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        
        //gets and stores the latest image from the camera
        PhotonPipelineResult camResult = camera.getLatestResult();
        if(camResult.hasTargets()) {
            //gets the clearest target in the latest image
            PhotonTrackedTarget seenTarget = camResult.getBestTarget();
            double yaw = seenTarget.getYaw();
            double pitch = seenTarget.getPitch();
            var camToTarget = seenTarget.getBestCameraToTarget();

            double targetToCamHeight = VisionConstants.kCamHeight - VisionConstants.kTagHeight;
            double frontToBackToTag = Math.tan(pitch) * targetToCamHeight;
            double sideToSideToTag;
            double turnToTag;


        }
     }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);
    }

} 
