package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX leftUpperMotor = new TalonFX(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID);
    private TalonFX leftLowerMotor = new TalonFX(ShooterConstants.LEFT_LOWER_FLYWHEEL_MOTOR_ID);

    private TalonFX rightUpperMotor = new TalonFX(ShooterConstants.RIGHT_UPPER_FLYWHEEL_MOTOR_ID);
    private TalonFX rightLowerMotor = new TalonFX(ShooterConstants.RIGHT_LOWER_FLYWHEEL_MOTOR_ID);

    private VelocityTorqueCurrentFOC req = new VelocityTorqueCurrentFOC(0);

    // Declared here to keep it tied to the shooter.
    public final Feeder feeder = new Feeder();

    public Shooter() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 7;
        conf.Slot0.kI = 2;
        conf.Slot0.kD = 0;

        conf.CurrentLimits.SupplyCurrentLimit = 50;
        conf.CurrentLimits.StatorCurrentLimit = 140;

        leftUpperMotor.getConfigurator().apply(conf);
        leftLowerMotor.getConfigurator().apply(conf);

        rightUpperMotor.getConfigurator().apply(conf);
        rightLowerMotor.getConfigurator().apply(conf);
    }

    @Override
    public void periodic() {
        StatusSignal.refreshAll(leftFlywheelVelocitySignal, rightFlywheelVelocitySignal);

        SmartDashboard.putNumber("Shooter/left flywheel velocity", leftUpperMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/right flywheel velocity", -rightUpperMotor.getVelocity().getValueAsDouble());
    }

    public void setVelocity(AngularVelocity velocity) {
        leftUpperMotor.setControl(req.withVelocity(velocity));
        leftLowerMotor.setControl(req.withVelocity(velocity));

        rightUpperMotor.setControl(req.withVelocity(velocity.unaryMinus()));
        rightLowerMotor.setControl(req.withVelocity(velocity.unaryMinus()));
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    public Command setVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
        return run(() -> setVelocity(velocitySupplier.get()));
    }

    public Command coastCommand() {
        return runOnce(() -> {
            leftUpperMotor.setControl(new CoastOut());
            rightUpperMotor.setControl(new CoastOut());
        });
    }

    /**
     * Gets the highest velocity of the two motors, as to not fail if a motor fails
     */
    private StatusSignal<AngularVelocity> leftFlywheelVelocitySignal = leftUpperMotor.getVelocity();
    private StatusSignal<AngularVelocity> rightFlywheelVelocitySignal = rightUpperMotor.getVelocity();
    public AngularVelocity getVelocity() {
        double leftVelocity = leftFlywheelVelocitySignal.getValueAsDouble();
        double rightVelocity = -rightFlywheelVelocitySignal.getValueAsDouble();

        return RotationsPerSecond.of(Math.max(leftVelocity, rightVelocity));
    }
}