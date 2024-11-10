package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonFactory {
    private ChoreoTrajectory trajectory;
    private CommandSwerveDrivetrain drivetrain;

    public Command getAutonomousCommand() {
        var thetaController = new PIDController(Constants.AutoConstants.KPTHETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        drivetrain.seedFieldRelative(trajectory.getInitialPose());

        Command swerveCommand = Choreo.choreoSwerveCommand(
            trajectory, // Choreo trajectory from above
            drivetrain::getPose, // A function that returns the current field-relative pose of the robot: your
                                // wheel or vision odometry
            new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0.0, 0.0), // PIDController for field-relative X
                                                                                    // translation (input: X error in meters,
                                                                                    // output: m/s).
            new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0.0, 0.0), // PIDController for field-relative Y
                                                                                    // translation (input: Y error in meters,
                                                                                    // output: m/s).
            thetaController, // PID constants to correct for rotation

            (ChassisSpeeds speeds) -> drivetrain.driveChassisSpeedsRobotCentric(speeds),

            () -> {return true;}, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)

            drivetrain // The subsystem(s) to require, typically your drive subsystem only
        );

        return swerveCommand;
    
    }
 
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
