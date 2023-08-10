package frc.robot.autostep;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.HotPID;
import frc.robot.Robot;

public class NavxPIDLevel extends AutoStep {

    public AHRS navx;
    public float degree;
    public float speed;
    public DriveTrain drivetrain;
    public boolean didClimb;
    public HotPID navxPID = new HotPID("navx PID", 0.01f, 0.0f, 0.003f);
    // public HotPID navxPID = new HotPID("navx PID", 0.01f, 0.0f, 0.005f);

    public Timer balanceTimer;
    public boolean timerStarted = false;
    

    public NavxPIDLevel(AHRS navx, DriveTrain drivetrain) {
        this.navx = navx;
        this.drivetrain = drivetrain;
    }

    public void Begin() {
        balanceTimer = new Timer();
        balanceTimer.start();
    }

    public void Update() {
        navxPID.setpoint = 0.0f;
        System.out.println(navx.getPitch());
        speed = (float)navxPID.Calculate(navx.getPitch());

        if ((Math.abs(speed) > 0.2f)){
            speed = 0.2f*speed/Math.abs(speed);
        }
        // if(Math.abs(navx.getPitch())<5.0f){
        // balanceTimer.reset();
        // balanceTimer.start();
        //     speed = 0.0f;
        //     timerStarted = true;
        // }
        // if(Math.abs((navx.getPitch()+navx.getRawGyroX())) < (Math.abs(navx.getPitch())-0.3f)){
        //     balanceTimer.reset();
        //     balanceTimer.start();
        //         speed = 0.0f;
        //         timerStarted = true;
        //     }


        // if(balanceTimer.get()<0.5f&& timerStarted){
        //     speed = 0.0f;
        // }
        if(Math.abs(navx.getPitch())< 5.0f){
            speed = 0.0f;
        }
        drivetrain.SetBothSpeed(-speed);

        // if (navx.getPitch() > 10) {
        //     didClimb = true;
        // }
        // if (didClimb && navx.getPitch() < 3) {
        //     isDone = true;
        //     drivetrain.SetBothSpeed(0);

        // }
      
    }
}
