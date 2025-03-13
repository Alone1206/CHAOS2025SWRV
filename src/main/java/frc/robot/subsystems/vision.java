// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Microseconds;
// import static edu.wpi.first.units.Units.Milliseconds;
// import static edu.wpi.first.units.Units.Seconds;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTablesJNI;
// import edu.wpi.first.wpilibj.Alert;
// import edu.wpi.first.wpilibj.Alert.AlertType;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;
// import java.awt.Desktop;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;
// import java.util.function.Supplier;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.PhotonUtils;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import swervelib.SwerveDrive;
// import swervelib.telemetry.SwerveDriveTelemetry;


// /**
//  * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
//  * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
//  */
// public class vision {

//   PhotonCamera camera = new PhotonCamera("FHD_Webcam");

//   public void updateResult() {
// //       var result = camera.getLatestResult(); // Now it's valid inside a method
// //       boolean hasTargets = result.hasTargets();
// //       PhotonTrackedTarget target = result.getBestTarget();
// //       int targetID = target.getFiducialId();
// // System.out.println(hasTargets);
// // System.out.println(targetID);
// boolean targetVisible = false;

//         double targetYaw = 0.0;

//         var results = camera.getAllUnreadResults();

//         if (!results.isEmpty()) {

//             // Camera processed a new frame since last

//             // Get the last one in the list.

//             var result = results.get(results.size() - 1);

//             if (result.hasTargets()) {

//                 // At least one AprilTag was seen by the camera

//                 for (var target : result.getTargets()) {

//                   int targetID = target.getFiducialId();


//                         targetYaw = target.getYaw();

//                         targetVisible = true;
//                         System.out.println(targetVisible);
//                         System.out.println(targetID);
//                         System.out.println(targetYaw);


                    

//                 }

//             }

//         }

// }
// }
