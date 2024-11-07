package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonFactory {
    private ChoreoTrajectory trajectory;
    private CommandSwerveDrivetrain drivetrain;

    public Command getAutonomousCommand() {
    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    drivetrain.resetOdometry(trajectory.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
        trajectory, // Choreo trajectory from above
        drivetrain::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> m_robotDrive.driveChassisSpeeds( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false),
        true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        m_robotDrive // The subsystem(s) to require, typically your drive subsystem only
    );

    public AutonFactory(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        ChoreoTrajectory trajectory =  Choreo.getTrajectory("Straight_Line_Test");
        if (trajectory == null) {
            return;
        } else {
            this.trajectory = trajectory;
        }
    }
}
