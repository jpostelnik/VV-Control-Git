package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

import com.qualcomm.robotcore.util.Range;

import org.ejml.simple.SimpleMatrix;

public class MotionProfiling {
    private double currentPower, maxPower;
    private SimpleMatrix target;
    private Kinematics kinematic;

    private static final double STEP = 0.15;
    private static final double MIN_POWER = 0.05;

    public MotionProfiling(double maxPower, Kinematics kinematic, SimpleMatrix target) {
        currentPower = MIN_POWER;
        this.kinematic = kinematic;
        this.maxPower = maxPower;
        this.target = target;
    }

    public double power() {
        currentPower += STEP;

        return currentPower > maxPower ? maxPower : currentPower;
    }

    public double power(SimpleMatrix dir , double percent ) {
        if (percent < 0.25) {
            currentPower = Range.clip(currentPower + STEP, MIN_POWER, maxPower);
        } else if (percent > 0.65) {
            currentPower = Range.clip(currentPower - STEP, MIN_POWER, maxPower);
        } else {
            currentPower = maxPower;
        }
        System.out.println("current power = "+currentPower);
        return currentPower;
    }

}
