package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonvisionSim {

    private PhotonCameraSim cameraSim;
    private SimCameraProperties cameraProperties;

    public PhotonvisionSim(Photonvision camera) {
        this.cameraProperties = new SimCameraProperties();
        this.cameraSim = new PhotonCameraSim(camera.getCamera(), this.cameraProperties, camera.getFieldLayout());
    }

    /**
     * Return the PhotonCameraSim object created by this class.
     * Used to setup the simulation.
     * @return The PhotonCameraSim object created by this class.
     */
    public PhotonCameraSim getCameraSim() {return this.cameraSim;}
}
