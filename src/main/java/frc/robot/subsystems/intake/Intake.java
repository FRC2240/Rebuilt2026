package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    public final IntakePivot pivot = new IntakePivot();

    private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);

    public Intake() {
    
         TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 1;

        conf.CurrentLimits.SupplyCurrentLimit = 50;
        conf.CurrentLimits.StatorCurrentLimit = 100;

        intakeMotor.getConfigurator().apply(conf);

    }

    public void setVelocity(AngularVelocity velocity) {
        intakeMotor.setControl(request.withVelocity(velocity));
    }
      
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", intakeMotor.getPosition().getValueAsDouble());
    }

    public Command setIntakeVelocity(AngularVelocity velocity) {
        return runOnce(() -> intakeMotor.setControl(new VelocityTorqueCurrentFOC(velocity)));
    }

    public Command setIntakeVelocity(Supplier<AngularVelocity> velocity) {
        return runOnce(() -> setVelocity(velocity.get()));
    }

    public Command enableIntakeCommand() {
        // Runs after setting control to prevent the default (enable) commmand from
        // being called until desired
        return setIntakeVelocity(IntakeConstants.INTAKE_VELOCITY).andThen(run(() -> {
        }));
    }

    public Command disableIntakeCommand() {
        return setIntakeVelocity(RotationsPerSecond.of(0)).andThen(run(() -> {
        }));
    }

    public Command reverseIntakeCommand() {
        return setIntakeVelocity(IntakeConstants.INTAKE_VELOCITY.unaryMinus()).andThen(run(() -> {
        }));
    }
}