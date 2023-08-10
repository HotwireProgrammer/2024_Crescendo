
package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.DriveTrain;
import frc.robot.HotPID;

public class NavxTurnPID extends AutoStep {
    private boolean toggle = false;
    public AHRS navx;
    public float turnDegree;
    public double speed;
    public float goodEnoughDeg;
    public int direction;

    public DriveTrain driveTrain;
    public HotPID navxPID;

    public NavxTurnPID(DriveTrain driveTrain, AHRS navx, float turnDegree, float goodEnoughDeg, HotPID navxPID) {
        super();
        this.navx = navx;
        this.turnDegree = turnDegree;
        this.driveTrain = driveTrain;
        this.goodEnoughDeg = goodEnoughDeg;
        this.navxPID = navxPID;
    }

    public void Begin() {
        System.out.println("RAN BEGIN");

    }

    public void Update() {
        float degreeDifference = Math.abs(navx.getYaw() - turnDegree);
        if (degreeDifference < goodEnoughDeg) {
            driveTrain.SetLeftSpeed(0);
            driveTrain.SetRightSpeed(0);
            if (driveTrain.getEncoderSpeed() == 0) {
                isDone = true;
            }
        }

        float diff = (turnDegree - navx.getYaw());
        direction = (int) ((Math.abs(diff)) / diff);
        speed = navxPID.Calculate(diff);

        System.out.println(speed + " speeeeeeedddddddddddd");
        if (speed > 0) {
            if (speed < 0.15f) {
                speed = 0.1f;// 0.15
            }
            System.out.println("ran");
            driveTrain.SetLeftSpeed((float) speed * direction);
            driveTrain.SetRightSpeed((float) -speed * direction);
            if (!toggle) {
                toggle = true;
                navxPID.reset();
            }
        } else {
            if (toggle) {
                toggle = false;
                navxPID.reset();
            }
            if (speed > -0.15f) {
                speed = -0.1f;// 0.15
            }
            System.out.println("run");
            driveTrain.SetLeftSpeed((float) -speed * direction);
            driveTrain.SetRightSpeed((float) speed * direction);
        }

        // HotPID.navxPID()
        System.out.println(navx.getYaw());
        // float degreeDifference = Math.abs(navx.getYaw() - turnDegree);
        // float goodEnoughDeg = 5.0f;
        // if (degreeDifference < goodEnoughDeg) {
        // driveTrain.SetLeftSpeed(0);
        // driveTrain.SetRightSpeed(0);
        // if(driveTrain.GetEncoderSpeed() == 0){
        // isDone = true;
        // }
        // }
    }
}