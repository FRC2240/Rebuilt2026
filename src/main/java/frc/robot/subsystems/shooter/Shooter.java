package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX leftFlywheelMotor = new TalonFX(ShooterConstants.LEFT_FLYWHEEL_MOTOR_ID);
    private TalonFX rightFlywheelMotor = new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_MOTOR_ID);

    private StatusSignal<AngularVelocity> leftMotorVelocity = leftFlywheelMotor.getVelocity();
    private StatusSignal<AngularVelocity> rightMotorVelocity = rightFlywheelMotor.getVelocity();

    private VelocityTorqueCurrentFOC req = new VelocityTorqueCurrentFOC(0);

    // Declared here to keep it tied to the shooter.
    public final Feeder feeder = new Feeder();

    public Shooter() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 5;
        conf.Slot0.kI = 0;
        conf.Slot0.kD = 0;

        leftFlywheelMotor.getConfigurator().apply(conf);
        rightFlywheelMotor.getConfigurator().apply(conf);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Current Velocity", getVelocity().in(RotationsPerSecond));
    }

    public void setVelocity(AngularVelocity velocity) {
        leftFlywheelMotor.setControl(req.withVelocity(velocity));
        rightFlywheelMotor.setControl(req.withVelocity(velocity));
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    public Command setVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
        return run(() -> setVelocity(velocitySupplier.get()));
    }

    /**
     * Gets the highest velocity of the two motors, as to not fail if a motor fails
     */
    public AngularVelocity getVelocity() {
        double leftVelocity = leftFlywheelMotor.getVelocity().getValueAsDouble();
        double rightVelocity = rightFlywheelMotor.getVelocity().getValueAsDouble();

        return RotationsPerSecond.of(Math.max(leftVelocity, rightVelocity));
    }
}
