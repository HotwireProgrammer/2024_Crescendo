package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.DriveTrain;
import frc.robot.swerve.DriveSubsystem;

public class NavxTurn extends AutoStep {

    public AHRS navx;
    public float turnDegree;
    public float speed;
    public float goodEnoughDeg;

    public DriveSubsystem driveTrain;

    public NavxTurn(DriveSubsystem driveTrain, AHRS navx, float turnDegree, float speed, float goodEnoughDeg) {
        super();
        this.navx = navx;
        this.speed = speed;
        this.turnDegree = turnDegree;
        this.driveTrain = driveTrain;
        this.goodEnoughDeg = goodEnoughDeg;
    }

    public void Begin() { }

    public void Update() {
        System.out.println(navx.getYaw());
        float degreeDifference = Math.abs(navx.getYaw() - turnDegree);

        driveTrain.GoToRotation(turnDegree);

        //float goodEnoughDeg = 5.0f;
        if (degreeDifference < goodEnoughDeg) {
            driveTrain.drive(0, 0, 0, true, true);
            isDone = true;
        }
    }
}