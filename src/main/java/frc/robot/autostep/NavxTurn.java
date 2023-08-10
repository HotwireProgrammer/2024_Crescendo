package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.DriveTrain;

public class NavxTurn extends AutoStep {

    public AHRS navx;
    public float turnDegree;
    public float speed;
    public float goodEnoughDeg;

    public DriveTrain driveTrain;

    public NavxTurn(DriveTrain driveTrain, AHRS navx, float turnDegree, float speed, float goodEnoughDeg) {
        super();
        this.navx = navx;
        this.speed = speed;
        this.turnDegree = turnDegree;
        this.driveTrain = driveTrain;
        this.goodEnoughDeg = goodEnoughDeg;
    }

    public void Begin() {
        float diff = (turnDegree - navx.getYaw());
        int direction = (int) ((Math.abs(diff)) / diff);

        driveTrain.SetLeftSpeed(speed * direction);
        driveTrain.SetRightSpeed(-speed * direction);
    }

    public void Update() {
        System.out.println(navx.getYaw());
        float degreeDifference = Math.abs(navx.getYaw() - turnDegree);
        //float goodEnoughDeg = 5.0f;
        if (degreeDifference < goodEnoughDeg) {
            driveTrain.SetLeftSpeed(0);
            driveTrain.SetRightSpeed(0);
            if (driveTrain.getEncoderSpeed() == 0) {
                isDone = true;
            }
        }else{
            float diff = (turnDegree - navx.getYaw());
            int direction = (int) ((Math.abs(diff)) / diff);
    
            driveTrain.SetLeftSpeed(speed * direction);
            driveTrain.SetRightSpeed(-speed * direction);
        }
    }
}