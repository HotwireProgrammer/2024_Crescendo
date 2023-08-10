package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Shooter;

public class ShooterRev extends AutoStep {

    public Shooter shooter;
    public double rpmTarget;

    public ShooterRev(Shooter shooter, double rpmTarget) {
        super();
        this.shooter = shooter;
        this.rpmTarget = rpmTarget;
    }

    public void Begin() {
        shooter.rpmTarget = rpmTarget;
    }

    public void Update() {
        isDone = true;
    }
}