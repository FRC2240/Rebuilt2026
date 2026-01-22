package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Field;

public class Shooter extends SubsystemBase{

    private TalonFX flywheelLeftMotor = new TalonFX(ShooterConstants.FLYWHEEL_LEFT_MOTOR_ID);
    private TalonFX flywheelRightMotor = new TalonFX(ShooterConstants.FLYWHEEL_RIGHT_MOTOR_ID);

    private ShooterState currState;

    public Shooter() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 10;
        conf.Slot0.kI = 6;
        conf.Slot0.kD = 6;


        flywheelLeftMotor.getConfigurator().apply(conf);

        conf.Slot0.kP = 10;
        conf.Slot0.kI = 6;
        conf.Slot0.kD = 6;


        flywheelRightMotor.getConfigurator().apply(conf);
    }

    public ShooterState getCurrentState() {
        return currState;
    }

    // setOutput() takes values from -1 to 1
    // This is for control of both motors at the same output.
    public void setOutput(double spd) {
        flywheelLeftMotor.set(spd);
        flywheelRightMotor.set(spd);
    }

    // This is used by set state for both motors to be given seperate outputs.
    public void setOutput(double spd, String dir) {
        switch (dir) {
            case "left":
                flywheelLeftMotor.set(spd);
            case "right":
                flywheelRightMotor.set(spd);
            default:
                System.out.println("Speed defaulted to: " + flywheelLeftMotor.get());
                flywheelLeftMotor.set(spd);
                flywheelRightMotor.set(spd);
        }
    }

    // This is for control of motors with differing outputs, for both to use one output see setOutput(spd).
    public void setState(ShooterState state) {
        setOutput(state.getLeftSpeed(), "left");
        setOutput(state.getRightSpeed(), "right");
        currState = state;
    }

    public double getOutput(String dir) {
        switch (dir) {
            case "left":
                return flywheelLeftMotor.get();
            case "right":
                return flywheelRightMotor.get();
            default:
                return -99;
        }
    }

    public Boolean canShoot() { // When calling this wrap it in an if(isfacinghub) { canShoot() } or do && isfacinghub
        // shooter velocity(prob some math to know if it can hit), hub is active
        //Math for if shooter velocity is good is needed
        Field.isHubActive();
        return true;
    }

    //TODO Implement once you know the robot architecture
    public LinearVelocity getBallVelocity() {
        //You will likely need to get the current flywheel rpm
        //this could also be done via a regression
        //switch for l/r/both

        return null;
    }

    // Returns target state for shooter
    public ShooterState calculateShooterState(double dist /* In Meters */) {
        // If the shot is not possible(too clos/too far), return the current state(aka no change) and do something to alert the driver(controller vibrate?)

        //We could do a gradient where depending how far we are into the band(dist from hub, that is achieveable) we ramp up the motor output
        //We could do regressions
        //We could do Kinematics
        return new ShooterState(0, 0);
    }
}
