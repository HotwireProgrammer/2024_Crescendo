package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.DriveTrain;

public class NavxDriveUntil extends AutoStep {

    public AHRS navx;
    public float degree;
    public float speed;
    public DriveTrain drivetrain;
    public boolean didClimb;

    public NavxDriveUntil(AHRS navx, float degree, float speed, DriveTrain drivetrain) {
        this.navx = navx;
        this.degree = degree;
        this.speed = speed;
        this.drivetrain = drivetrain;
    }

    public void Begin() {

    }

    public void Update() {
        System.out.println(navx.getPitch());

        drivetrain.SetBothSpeed(speed);

        if (navx.getPitch() > 10) {
            didClimb = true;
        }
        if (didClimb && navx.getPitch() < 3) {
            isDone = true;
            drivetrain.SetBothSpeed(0);

        }
      
    }
}
