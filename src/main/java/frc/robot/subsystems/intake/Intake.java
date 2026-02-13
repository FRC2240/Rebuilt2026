package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    public Intake() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 1;

        pivotMotor.getConfigurator().apply(conf);
        intakeMotor.getConfigurator().apply(conf);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pivotMotor Velocity", pivotMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("intakeMotor Velocity", pivotMotor.getVelocity().getValueAsDouble());
    }

    public Command setIntakeVelocity(AngularVelocity velocity) {
        return runOnce(() -> intakeMotor.setControl(new VelocityTorqueCurrentFOC(velocity)));
    }

    public Command extendIntakeCommand() {
        return runOnce(() -> pivotMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.EXTENDED_POSITION)));
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