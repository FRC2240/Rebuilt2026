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
    private boolean shootOverride = false;

    public boolean canHit = true;
    private boolean isShooting = false;

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
        return this.currState;
    }

    // setOutput() takes values from -1 to 1
    // This is for control of both motors at the same output. Please use setState() for control/command implementation
    public void setOutput(double spd) {
        flywheelLeftMotor.set(spd);
        flywheelRightMotor.set(spd);
    }

    // Sets output along with current state this is what should be turned into a command
    public void setState(ShooterState state) {
        setOutput(state.getSpeed());
        this.currState = state;
    }

    public double getOutput(String dir) {
        if(flywheelLeftMotor.get() >= flywheelRightMotor.get()) {
            return flywheelLeftMotor.get();
        }
        else {
            return flywheelRightMotor.get();
        }
        // This has no Check if both go down, but at that point it should be obvious to the drive team.
    }

    public void shoot() {
        //this may be moved to indexer
    }

    public Boolean canShoot() { // When calling this wrap it in an if(isfacinghub) { canShoot() } or do && isfacinghub
        if(this.shootOverride) return true;
        // You must run setState(calculateShooterState(double)) before running this command as it will update the current state
        // It will determine if the shot itself is possible.
        // The following line checks this 
        if(!this.currState.canHit) return false;

        return Field.isHubActive();
        // We may want this to vibrate the controller, this functionality could be added to robot container, when bool is false
    }

    public void canShootOverride() {
        this.shootOverride = !this.shootOverride;
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

        //TODO: add calculations so that canHit will change if need be
        

        if(!this.canHit) {
            this.canHit = true; // this does not affect the ShooterState and whether or not it can hit, it just resets to default
            return new ShooterState(this.currState.getSpeed(), false);
        }

        return new ShooterState(0);
    }

    public boolean isShooting() {
        return this.isShooting;
    }
}
