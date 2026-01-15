package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdRunner {
    private Drivetrain drivetrain;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public SysIdRunner(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    state -> SmartDashboard.putString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> drivetrain.setControl(translationCharacterization.withVolts(output)),
                    this::logSysIdTranslation,
                    drivetrain));

    private void logSysIdTranslation(SysIdRoutineLog log) {
        var modules = drivetrain.getModules();
        for (int i = 0; i < 4; i++) {
            var module = modules[i];
            TalonFX driveMotor = module.getDriveMotor();
            log.motor("drive-motor-" + i).voltage(driveMotor.getMotorVoltage().getValue())
                    .angularPosition(driveMotor.getPosition().getValue())
                    .angularVelocity(driveMotor.getVelocity().getValue());
        }
    }

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    state -> SmartDashboard.putString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> drivetrain.setControl(steerCharacterization.withVolts(volts)),
                    this::logSysIdSteer,
                    drivetrain));

    private void logSysIdSteer(SysIdRoutineLog log) {
        var modules = drivetrain.getModules();
        for (int i = 0; i < 4; i++) {
            var module = modules[i];
            TalonFX angleMotor = module.getSteerMotor();
            CANcoder encoder = module.getEncoder();

            log.motor("steer-motor-" + i).voltage(angleMotor.getMotorVoltage().getValue())
                    .angularPosition(encoder.getPosition().getValue())
                    .angularVelocity(encoder.getVelocity().getValue());
        }
    }

    /*
     * SysId routine for characterizing rotation. This is used to find PID gains for
     * the FieldCentricFacingAngle HeadingController. See the documentation of
     * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    state -> SmartDashboard.putString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        drivetrain.setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SmartDashboard.putNumber("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    drivetrain));

    /* The SysId routine to test */
    private SysIdRoutine sysIdRoutineToApply = sysIdRoutineTranslation;

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }
}
