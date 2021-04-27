package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math;


public class basicMathCalls {

    private static final double TICKS_PER_REV = 560;
    private static final double DRIVETRAIN_WHEEL_DIAMTER = 4;
    private static final double DRIVETRAIN_GEAR_RATIO = 1;
    private static final double LINEAR_TO_TICKS = TICKS_PER_REV / (Math.PI * DRIVETRAIN_WHEEL_DIAMTER * DRIVETRAIN_GEAR_RATIO);
    //    private static final double radius = 6.5;
    private static final double STRAFE_COEFFICIENT = 1.20;
    private static final double DEAD_WHEEL_DIAMTER = 2;
    private static final double DEAD_WHEEL_TICKS_PER_REV = 4096;
    private static final double DEAD_WHEEL_TO_TICKS = DEAD_WHEEL_TICKS_PER_REV / (Math.PI * DEAD_WHEEL_DIAMTER); //652.229


    public static double getSecondPower(double degrees, double power) {

        double secondaryPower;

        double degreesProportional = degrees / 45;
        if (degreesProportional < -3) {
            secondaryPower = (degreesProportional + 3) * power * -1;
        } else if ((degreesProportional < -2)) {
            secondaryPower = (degreesProportional + 2) * power;
        } else if (degreesProportional < -1) {
            secondaryPower = (degreesProportional + 1) * power;
        } else if (degreesProportional < 0) {
            secondaryPower = (degreesProportional) * power * -1;
        } else if (degreesProportional < 1) {
            secondaryPower = degreesProportional * power * -1;
        } else if (degreesProportional < 2) {
            secondaryPower = (degreesProportional - 1) * power;
        } else if (degreesProportional < 3) {
            secondaryPower = (degreesProportional - 2) * power;
        } else if (degreesProportional < 4) {
            secondaryPower = (degreesProportional - 3) * power * -1;
        } else secondaryPower = 0;


//        if (degrees < 90) {
//
//            secondaryPower = (degrees / 45) * power;
//
//        } else if (degrees > 90) {
//            secondaryPower = -1 * (degrees - 135) * power;
//        } else if (degrees < -90) {
//            secondaryPower = (degrees + 135) * power * (-1);
//        } else if (degrees > -90) {
//            secondaryPower = -1 * (degrees + 45) * power * (-1);
//        } else secondaryPower = 0;
        secondaryPower *= 0.1;

        return secondaryPower;
    }

    public static double getAngleDegrees(double xDistance, double yDistance) {
        double angle = Math.atan2(xDistance, yDistance);
        return Math.toDegrees(angle);

    }

    public static int getTicksMotor(double distance, String type) {
        if (type.contains("linear")) {
            return (int) (distance * LINEAR_TO_TICKS);
        } else if (type.contains("strafe")) {
            return (int) (distance * LINEAR_TO_TICKS * STRAFE_COEFFICIENT);
        }

        return 0;
    }

    public static int getDeadWheelTicks(double distance) {
        return (int) (distance * DEAD_WHEEL_TO_TICKS);
    }

    public static double ticksToInch(double ticks){
        return (ticks/DEAD_WHEEL_TO_TICKS);
    }
}
