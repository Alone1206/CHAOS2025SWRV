// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Meter;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.swervedrive.Vision;

// import java.io.File;
// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import swervelib.parser.SwerveParser;
// import swervelib.telemetry.SwerveDriveTelemetry;
// import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
// import swervelib.SwerveDrive;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.PathPlannerPath;

// public class SwerveSubsystem extends SubsystemBase {

//     File directory = new File(Filesystem.getDeployDirectory(), "swerve");
//     SwerveDrive swerveDrive;
//     private final Vision vision;

//     public SwerveSubsystem() {
//         /* SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; */

//         try {
//             swerveDrive = new SwerveParser(directory).createSwerveDrive(
//                 Constants.maximumSpeed,
//                 new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
//             );
//         } catch (Exception e) {
//             throw new RuntimeException(e);
//         }

//         // Initialize Vision subsystem
//         vision = new Vision(() -> swerveDrive.getPose(), swerveDrive.field);
//         setupPathPlanner();
//     }

//     @Override
//     public void periodic() {
//         // Update vision-based pose estimation
//         vision.updatePoseEstimation(swerveDrive);
//         vision.updateVisionField();
        
//         // Update swerve drive odometry
//         swerveDrive.updateOdometry();
//     }

//     @Override
//     public void simulationPeriodic() {
//         // Update vision simulation
//         if (Robot.isSimulation() && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
//             vision.getVisionSim().update(swerveDrive.getSimulationDriveTrainPose().get());
//         }
//     }

//     public SwerveDrive getSwerveDrive() {
//         return swerveDrive;
//     }

//     public void driveFieldOriented(ChassisSpeeds velocity) {
//         swerveDrive.driveFieldOriented(velocity);
//     }

//     public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
//         return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
//     }

//     private void setupPathPlanner() {
//         try {
//             RobotConfig config = RobotConfig.fromGUISettings();
//             final boolean enableFeedforward = true;

//             AutoBuilder.configure(
//                 swerveDrive::getPose,
//                 swerveDrive::resetOdometry,
//                 swerveDrive::getRobotVelocity,
//                 (speedsRobotRelative, moduleFeedForwards) -> {
//                     if (enableFeedforward) {
//                         swerveDrive.drive(
//                             speedsRobotRelative,
//                             swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
//                             moduleFeedForwards.linearForces()
//                         );
//                     } else {
//                         swerveDrive.setChassisSpeeds(speedsRobotRelative);
//                     }
//                 },
//                 new PPHolonomicDriveController(
//                     new PIDConstants(5.0, 0.0, 0.0),
//                     new PIDConstants(5.0, 0.0, 0.0)
//                 ),
//                 config,
//                 () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
//                 this
//             );
//         } catch (Exception e) {
//             e.printStackTrace();
//         }
//     }

//     public Command getAutonomousCommand(String pathName) {
//         return new PathPlannerAuto(pathName);
//     }

//     // Additional vision methods
//     public double getDistanceFromAprilTag(int id) {
//         return vision.getDistanceFromAprilTag(id);
//     }

//     public Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
//         return Vision.getAprilTagPose(aprilTag, robotOffset);
//     }
// }