package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> currentStdDevs = VisionConstants.kSingleTagStdDevs;

    public Vision() {
        camera = new PhotonCamera(VisionConstants.kCameraName);
        
        photonEstimator = new PhotonPoseEstimator(
            VisionConstants.kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            VisionConstants.kRobotToCam
        );
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Get the latest vision-estimated pose
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            visionEst = photonEstimator.update(result);
            updateEstimationStdDevs(visionEst, result.getTargets());
        }
        
        return visionEst;
    }

    /**
     * Get the standard deviations for the latest pose estimate
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            currentStdDevs = VisionConstants.kSingleTagStdDevs;
            return;
        }

        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Calculate average distance to visible tags
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            currentStdDevs = VisionConstants.kSingleTagStdDevs;
            return;
        }

        // Update standard deviations based on number of tags and distance
        avgDist /= numTags;
        if (numTags > 1) {
            estStdDevs = VisionConstants.kMultiTagStdDevs;
        }
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        currentStdDevs = estStdDevs;
    }
}