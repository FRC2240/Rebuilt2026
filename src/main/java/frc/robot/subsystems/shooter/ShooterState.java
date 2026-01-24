package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterState {

    private AngularVelocity speed;
    public boolean canHit = true;
    
    public ShooterState(AngularVelocity spd) {
        speed = spd;
    }
    
    public ShooterState(AngularVelocity spd, boolean isPossible) {
        speed = spd;
        canHit = isPossible;
    }

    public AngularVelocity getSpeed() {
        return speed;
    }
}
