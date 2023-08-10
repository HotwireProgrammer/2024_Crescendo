package frc.robot.autostep;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.Indexer;

public class EncoderForwardFeet extends AutoStep {

    public float adjuster = 11398;
    public float encoderTargetFeet;
    public float encoderTarget;
    public float speed;
    public double encoderStart;
    public float dir;

    public DriveTrain driveTrain;

    public EncoderForwardFeet(DriveTrain driveTrain, float encoderTargetFeet, float speed) {
        super();
        this.encoderTargetFeet = encoderTargetFeet;
        this.speed = speed;
        this.driveTrain = driveTrain;
    }

    public void Begin() {
        driveTrain.SetBothSpeed(speed);
        encoderStart = driveTrain.getEncoder();
        dir = speed / Math.abs(speed);
    }

    public void Update() {
        encoderTarget = encoderTargetFeet * adjuster;
        double currentError = Math.abs(encoderStart - driveTrain.getEncoder());
        if (encoderTarget-currentError < 48000) {
            driveTrain.SetBothSpeed(0.4f * dir);
        }
        if (currentError > encoderTarget) {
            isDone = true;
            driveTrain.SetBothSpeed(0.0f);
        }
        System.out.println(encoderStart - driveTrain.getEncoder());

    }
}