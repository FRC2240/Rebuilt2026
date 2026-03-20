package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunableNumber;

public class Shooter extends SubsystemBase {

    private TalonFX leftFlywheelMotor = new TalonFX(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID);
    private TalonFX leftFlywheelFollower = new TalonFX(ShooterConstants.LEFT_LOWER_FLYWHEEL_MOTOR_ID);

    private TalonFX rightFlywheelFollowerUpper = new TalonFX(ShooterConstants.RIGHT_UPPER_FLYWHEEL_MOTOR_ID);
    private TalonFX rightFlywheelFollowerLower = new TalonFX(ShooterConstants.RIGHT_LOWER_FLYWHEEL_MOTOR_ID);

    private VelocityTorqueCurrentFOC req = new VelocityTorqueCurrentFOC(0);

    private TunableNumber p = new TunableNumber("Shooter/PID/p", 8);
    private TunableNumber i = new TunableNumber("Shooter/PID/i", 2);
    private TunableNumber d = new TunableNumber("Shooter/PID/d", 0);
    
    // Declared here to keep it tied to the shooter.
    public final Feeder feeder = new Feeder();

    public Shooter() {
        applyConfiguration();
        p.addChangeListener((v) -> applyConfiguration());
        i.addChangeListener((v) -> applyConfiguration());
        d.addChangeListener((v) -> applyConfiguration());

        leftFlywheelFollower.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Aligned));

        rightFlywheelFollowerUpper.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));
        
        rightFlywheelFollowerLower.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void applyConfiguration() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = p.get();
        conf.Slot0.kI = i.get();
        conf.Slot0.kD = d.get();

        conf.CurrentLimits.SupplyCurrentLimit = 100;
        conf.CurrentLimits.StatorCurrentLimit = 140;

        leftFlywheelMotor.getConfigurator().apply(conf);
    }

    @Override
    public void periodic() {
        StatusSignal.refreshAll(leftFlywheelVelocitySignal);

        SmartDashboard.putNumber("Shooter/flywheel velocity", leftFlywheelMotor.getVelocity().getValueAsDouble());
    }

    public void setVelocity(AngularVelocity velocity) {
        leftFlywheelMotor.setControl(req.withVelocity(velocity));
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    public Command setVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
        return run(() -> setVelocity(velocitySupplier.get()));
    }

    public Command coastCommand() {
        return runOnce(() -> {
            leftFlywheelMotor.setControl(new CoastOut());
        });
    }

    /**
     * Gets the highest velocity of the two motors, as to not fail if a motor fails
     */
    private StatusSignal<AngularVelocity> leftFlywheelVelocitySignal = leftFlywheelMotor.getVelocity();


    public AngularVelocity getVelocity() {
        double leftVelocity = leftFlywheelVelocitySignal.getValueAsDouble();

        return RotationsPerSecond.of(leftVelocity);
    }
}