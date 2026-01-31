package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(ShooterConstants.LEFT_FEEDER_MOTOR_ID);
    private TalonFX rightMotor = new TalonFX(ShooterConstants.RIGHT_FEEDER_MOTOR_ID);

    private VelocityTorqueCurrentFOC req = new VelocityTorqueCurrentFOC(0);

    public Indexer() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 1;
        conf.Slot0.kI = 0;
        conf.Slot0.kD = 0;

        leftMotor.getConfigurator().apply(conf);
        rightMotor.getConfigurator().apply(conf);
        
        setDefaultCommand(disableCommand());
    }

    public void setVelocity(AngularVelocity velocity) {
        leftMotor.setControl(req.withVelocity(velocity));
        rightMotor.setControl(req.withVelocity(velocity));
    }

    /**
     * Starts the shooter indexer motors
     */
    public Command enableCommand() {
        return runOnce(() -> setVelocity(ShooterConstants.LAUNCH_MOTOR_OUTPUT)).andThen(run(() -> {}));
    }

    /**
     * Stops the shooter indexer motors. 
     */
    public Command disableCommand() {
        return runOnce(() -> setVelocity(RotationsPerSecond.of(0))).andThen(run(() -> {}));
    }
}
