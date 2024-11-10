package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // registerTelemetry((state) -> {
        //    state.
        // });
    } 

    public Pose2d getPose() {
        if (getState().Pose == null){
            return new Pose2d();
        }
        return getState().Pose;
    }

    public void resetHeading() {
        this.seedFieldRelative();
    }

    public void setBoost(boolean boost) {}

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void configureDefaultCommand(CommandXboxController driverController) {
        setDefaultCommand( // Drivetrain will execute this command periodically
            applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                            // negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }

    public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        drive.withVelocityX(chassisSpeeds.vxMetersPerSecond);
        drive.withVelocityY(chassisSpeeds.vyMetersPerSecond);
        drive.withRotationalRate(chassisSpeeds.omegaRadiansPerSecond);

        desiredChassisSpeeds = chassisSpeeds;

        this.setControl(m_requestToApply);
    }

    public void driveChassisSpeedsRobotCentric(ChassisSpeeds chassisSpeeds) {
        driveRobotCentric.withVelocityX(chassisSpeeds.vxMetersPerSecond);
        driveRobotCentric.withVelocityY(chassisSpeeds.vyMetersPerSecond);
        driveRobotCentric.withRotationalRate(chassisSpeeds.omegaRadiansPerSecond);

        desiredChassisSpeeds = chassisSpeeds;

        this.setControl(m_requestToApply);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(m_moduleStates);
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return m_moduleStates;
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        Logger.recordOutput("Drivetrain/Pose2d", getPose());
        Logger.recordOutput("Drivetrain/ChassisSpeeds", getChassisSpeeds());
        Logger.recordOutput("Drivetrain/DesiredChassisSpeeds", desiredChassisSpeeds);
        Logger.recordOutput("Drivetrain/ModuleStates", getSwerveModuleStates());

        //    ƪ(˘⌣˘)ʃ  

        SmartDashboard.putNumber("FL-CanCoder", super.getModule(0).getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("FR-CanCoder", super.getModule(1).getCANcoder().getAbsolutePosition().getValueAsDouble());        
        SmartDashboard.putNumber("RL-CanCoder", super.getModule(2).getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("RR-CanCoder", super.getModule(3).getCANcoder().getAbsolutePosition().getValueAsDouble());

        SmartDashboard.putNumber("FL-SteerPos", super.getModule(0).getSteerMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("FR-SteerPos", super.getModule(1).getSteerMotor().getPosition().getValueAsDouble());        
        SmartDashboard.putNumber("RL-SteerPos", super.getModule(2).getSteerMotor().getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RR-SteerPos❤️", super.getModule(3).getSteerMotor().getPosition().getValueAsDouble());

    }
}