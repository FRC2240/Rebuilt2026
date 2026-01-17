package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private TalonFX flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID);
    private TalonFX angleMotor = new TalonFX(ShooterConstants.ANGLE_MOTOR_ID);
    private TalonFX loaderMotor = new TalonFX(ShooterConstants.LOADER_MOTOR_ID); // This is the motor that pushes balls into the flywheel, maybe move to indexer

    private MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0);

    ShooterState currState;

    public Shooter() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 10;
        conf.Slot0.kI = 6;
        conf.Slot0.kD = 6;


        flywheelMotor.getConfigurator().apply(conf);

        conf.Slot0.kP = 10;
        conf.Slot0.kI = 6;
        conf.Slot0.kD = 6;


        loaderMotor.getConfigurator().apply(conf);

        conf.Slot0.kP = 10;
        conf.Slot0.kI = 6;
        conf.Slot0.kD = 6;
        conf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;


        angleMotor.getConfigurator().apply(conf);
    }

    public ShooterState getCurrentState() {
        return currState;
    }
    

    public void setAngle(Angle angle) {
        angleMotor.setControl(req.withPosition(angle));
    }

    // setOutput() takes values from -1 to 1
    public void setOutput(double spd) {
        flywheelMotor.set(spd);
    }

    public void setState(ShooterState state) {
        setAngle(state.getAngle());
        setOutput(state.getSpeed());
        currState = state;
    }

    public void shoot() {
        // Code for loader motor to feed the ball to flywheel will come when cad is done

    }



    //TODO Implement once you know the robot architecture
    public LinearVelocity getBallVelocity() {
        //You will likely need to get the current flywheel rpm

        return null;
    }

    // Returns target state for shooter
    public ShooterState calculateShooterState() {
        //Depending on the method, this will probably use Ball velocity, distance from hub, and potentially kinematics
        // It also might take in the current shooter state and adjust it before returning it
        return new ShooterState(ShooterConstants.MIN_ANGLE, 0);
    }

    //Deffered beacuse plan right now is a fixed Shooter Angle
    // Finds the angle of the physical shooter
    public Angle rotsToTrueDeg(Angle rotations) {
        //depends on the gear ratio for calculations
        return Degrees.of(360);
    }
    
}
