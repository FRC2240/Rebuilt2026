package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase{
    private TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);

    public IntakePivot() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 5;
        conf.Slot0.kI = 2;

        pivotMotor.getConfigurator().apply(conf);
        pivotMotor.setPosition(Rotations.of(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pivotMotor Velocity", pivotMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("intakeMotor Velocity", pivotMotor.getVelocity().getValueAsDouble());
    }

    public Command setPositionCommand(Angle position) {
        return runOnce(() -> pivotMotor.setControl(new PositionTorqueCurrentFOC(position))).andThen(run(() -> {}));
    }

    public Command extendCommand() {
        return setPositionCommand(IntakeConstants.PIVOT_EXTENDED_POSITION);
    }

    public Command contractCommand() {
        return setPositionCommand(Rotations.of(0));
    }

    public Command rampCommand() {
        return setPositionCommand(IntakeConstants.PIVOT_RAMP_POSITION);
    }
}
