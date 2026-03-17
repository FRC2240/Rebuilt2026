package frc.robot.subsystems.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.RobotPosition;

public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final Logger logger = new Logger();

    // Drive swerve request for Pathplanner control.
    private final SwerveRequest.ApplyRobotSpeeds ppDrive = new SwerveRequest.ApplyRobotSpeeds();

    public final DriveCommands commands;
    public Object DriveCommands;

    public Drivetrain(CommandXboxController controller) {
        super(TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);

        RobotPosition.init(this); // Initializes singleton RobotPosition class
        this.commands = new DriveCommands(this, controller);

        // Idle the swerve modules when disabled, ensuring the configured
        // neutral mode is applied to the drive motors when disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(applyRequest(() -> idle).ignoringDisable(true));

        registerTelemetry(logger::telemeterize);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        setupPathPlanner();
    }

    private void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    () -> this.getState().Speeds,
                    (speeds, feedforwards) -> setControl(ppDrive.withSpeeds(speeds)),
                    new PPHolonomicDriveController(
                            new PIDConstants(2.0, 0.0, 0.10), // Translation PID constants
                            new PIDConstants(2.0, 0.0, 0.10) // Rotation PID constants
                    ),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /*
         * Uses a notifier to run simulation at a faster rate so PID gains behave more
         * reasonably
         */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is
     *         empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public Command rezeroGyro() {
        return runOnce(() -> resetRotation(Rotation2d.kZero)).ignoringDisable(true);
    }
    
    public Pose2d getPose() {
        return getState().Pose;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }
}