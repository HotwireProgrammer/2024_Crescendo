package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Indexer;

public class EncoderForward extends AutoStep {

    public float encoderTarget;
    public float speed;
    public double encoderStart;
    public float dir;

    public DriveTrain driveTrain;

    public EncoderForward(DriveTrain driveTrain, float encoderTarget, float speed) {
        super();
        this.encoderTarget = encoderTarget;
        this.speed = speed;
        this.driveTrain = driveTrain;
    }

    public void Begin() {
        driveTrain.SetBothSpeed(speed);
        encoderStart = driveTrain.getEncoder();
        dir = speed / Math.abs(speed);
    }

    public void Update() {

        double currentError = Math.abs(encoderStart - driveTrain.getEncoder());

        if (encoderTarget-currentError < 48000) {
            driveTrain.SetBothSpeed(0.2f * dir);
        }
        if (currentError > encoderTarget) {
            isDone = true;
            driveTrain.SetBothSpeed(0.0f);
        }
        System.out.println(encoderStart - driveTrain.getEncoder());

    }
}