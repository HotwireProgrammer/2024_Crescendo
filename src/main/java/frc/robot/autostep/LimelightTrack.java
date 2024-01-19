package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Shooter;
import frc.robot.Limelight;

public class LimelightTrack extends AutoStep {

    public Timer limeTimer;
    public DriveTrain driveTrain;
    public Shooter shooter;
    public Limelight limelight;
    public double xAdjust;

    public LimelightTrack(DriveTrain driveTrain, Shooter shooter, Limelight limelight, double xAdjust) {
        super();
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.limelight = limelight;
        this.xAdjust = xAdjust;
    }

    public void Begin() {
        limeTimer = new Timer();
        limeTimer.reset();
        limeTimer.start();
    }

    public void Update() {
        

        if (limeTimer.get() > 1.5f) {
            isDone = true;
        }

        runShooter = true;


        //limelight.Position(driveTrain);
        if (limelight.OnTarget() && driveTrain.getEncoderSpeed() == 0) {
            limelight.reset();
            isDone = true;
        }
    }
}