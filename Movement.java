package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.exceptions.HeartBeatException;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.KalmanFilter;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.Kinematics;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.MotionProfiling;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.PIDController;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.basicMathCalls.ticksToInch;

public class Movement {

    private double watchDogExpiration;
    private final double EPSILON = 1;
    private final double WIDTH = 5;
    private final double HEIGHT = 5.5;

    private DcMotorEx rightFront, leftFront, rightRear, leftRear, shootR, shootL, intake, feeder;
    private DcMotorEx encX, encY;
    private BNO055IMU imu;
    private DcMotorEx[] driveTrain;


    private Kinematics robotKinematics = new Kinematics(WIDTH, HEIGHT);
    private ElapsedTime runtime = new ElapsedTime();
    private PIDController pid = new PIDController(runtime, 0.0004, 0, 0.0024);

    //KalmanFilter Filter
    //updates ever 50 miliseconds
    double dt = 0.025;
    SimpleMatrix F = new SimpleMatrix(new double[][]{
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    });

    SimpleMatrix G = new SimpleMatrix(new double[][]{
            {0.5 * dt * dt},
            {0.5 * dt * dt},
            {dt},
            {dt}
    });

    double sigma_a = 0.005;
    SimpleMatrix Q = G.mult(G.transpose()).scale(sigma_a * sigma_a);

    SimpleMatrix H = SimpleMatrix.identity(F.numRows());

    double positionVar = 0.005;
    double velocityVar = 0.001;

    SimpleMatrix R = new SimpleMatrix(new double[][]{
            {positionVar, 0, 0, 0},
            {0, positionVar, 0, 0},
            {0, 0, velocityVar, 0},
            {0, 0, 0, velocityVar}
    });


    double a_x = 20;
    double a_y = 20;

    SimpleMatrix u_u = new SimpleMatrix(new double[][]{
            {a_x},
            {a_y},
            {a_x},
            {a_y}
    });

    SimpleMatrix u_0 = new SimpleMatrix(new double[][]{
            {0},
            {0},
            {0},
            {0}
    });

    KalmanFilter robotKalmanFilter = new KalmanFilter(F, G, R, Q, H);


    /**
     * Move robot to desired position
     *
     * @param target  Destination in robot coordinate plane (x is side-to-side, y is forward-backward)
     * @param power   Max power to use (0-1)
     * @param runtime Internal system clock
     * @throws HeartBeatException thrown if fail safe is violated or mode changes to teleop
     */
    public void move(SimpleMatrix target, double power, ElapsedTime runtime) throws HeartBeatException {
        pid.reset(runtime);
        resetDeadWheels();
        MotionProfiling mp = new MotionProfiling(power, robotKinematics, target);

        double origDistanceToTarget = target.normF();
        double theta = Math.atan2(target.get(1), target.get(0));
        SimpleMatrix B = new SimpleMatrix(new double[][]{
                {dt * dt / 2 * Math.cos(theta), 0, 0, 0},
                {0, dt * dt / 2 * Math.sin(theta), 0, 0},
                {0, 0, dt * dt / 2 * Math.cos(theta), 0},
                {0, 0, 0, dt * dt / 2 * Math.sin(theta)}
        });

        setWatchDogExpiration(origDistanceToTarget / 10);

        SimpleMatrix x_0 = new SimpleMatrix(new double[][]{
                {0},
                {0},
                {0},
                {0}
        });
        SimpleMatrix p_0 = new SimpleMatrix(new double[][]{
                {0.1, 0, 0, 0},
                {0, 0.1, 0, 0},
                {0, 0, 0.1, 0},
                {0, 0, 0, 0.1}
        });
        robotKalmanFilter.setInitalPostion(x_0, p_0, B);

        SimpleMatrix updatedEst = x_0;

        double nextUpdateTime = runtime.milliseconds() + dt * 1000;
        double lastEstimateTime = runtime.seconds();
        double lastX = ticksToInch(encX.getCurrentPosition()),
                lastY = ticksToInch(encY.getCurrentPosition());

        while (true) {
            SimpleMatrix error = target.minus(position(updatedEst));
            double currentDistToTarget = error.normF();
            SimpleMatrix errorDir = error.scale(1 / currentDistToTarget);

            System.out.println(
                    "distance = " + currentDistToTarget +
                            "\nheading= " + Math.toDegrees(Math.atan2(errorDir.get(1), errorDir.get(0)))
            );
            if (currentDistToTarget < EPSILON) {
                break;
            }
            heartbeat();

            double currentPower = mp.power(errorDir, 1 - currentDistToTarget / origDistanceToTarget);
            SimpleMatrix Vnext = errorDir.scale(currentPower);
            SimpleMatrix w = robotKinematics.getWheelPower(Vnext.get(0), Vnext.get(1), 0);
            SimpleMatrix c = correctHeading(runtime);
            drive(w.plus(c));

            double sleepTime = nextUpdateTime - runtime.milliseconds();
            if (sleepTime > 0) {
                sleep((long) sleepTime);
            }
            nextUpdateTime += dt * 1000;

            double x = ticksToInch(encX.getCurrentPosition()),
                    y = ticksToInch(encY.getCurrentPosition());
            double dt_actual = runtime.seconds() - lastEstimateTime;
            lastEstimateTime = runtime.seconds();

            SimpleMatrix z_k = new SimpleMatrix(new double[][]{
                    {x},
                    {y},
                    {(x - lastX) / dt_actual},
                    {(y - lastY) / dt_actual}
            });
            lastX = x;
            lastY = y;

            SimpleMatrix v = velocity(z_k);
            double speed = v.normF();
            SimpleMatrix u = estimateControlInput(speed, 50);
            updatedEst = robotKalmanFilter.update(z_k, u);
//             updatedEst = z_k;

            System.out.println("dt_actual = " + dt_actual);
            System.out.println("speed = " + speed);
            System.out.println("z_k = " + z_k.transpose());
            System.out.println("error= " + error.transpose());
            System.out.println("error norm=" + error.normF());
//            System.out.println("updatedEst = " + updatedEst.transpose());
            telemetry.update();
        }

        halt();
    }

    private SimpleMatrix estimateControlInput(double speed, int steadyStateSpeed) {
        SimpleMatrix u;
        if (speed < steadyStateSpeed) {
            u = u_u;
        } else {
            u = u_0;
        }
        return u;
    }

    private void drive(double currentPower) {
        for (DcMotorEx motor : driveTrain) {
            motor.setPower(currentPower);
        }
    }

    private void drive(SimpleMatrix w) {
        rightFront.setPower(w.get(0));
        leftFront.setPower(w.get(1));
        leftRear.setPower(w.get(2));
        rightRear.setPower(w.get(3));
    }

    private SimpleMatrix position(SimpleMatrix updatedEst) {
        return updatedEst.extractMatrix(0, 2, 0, 1);
    }

    private SimpleMatrix velocity(SimpleMatrix updatedEst) {
        return updatedEst.extractMatrix(2, 4, 0, 1);
    }

    private SimpleMatrix correctHeading(ElapsedTime runtime) {
        double correction = pid.correction(getCurrentAngle(), runtime);
        SimpleMatrix correctionMatrix = new SimpleMatrix(new double[][]{
                {-correction},
                {correction},
                {correction},
                {-correction}
        });
        return correctionMatrix;
    }


    private double distance(SimpleMatrix x1, SimpleMatrix x2) {
        return x1.minus(x2).normF();
    }

    protected void halt() {
        drive(0);
    }

    public void setWatchDogExpiration(double watchDogExpiration) {
        this.watchDogExpiration = watchDogExpiration + runtime.seconds();
    }

    private void heartbeat() throws HeartBeatException {
        if (!opModeIsActive()) {
            System.out.println("op mode failed");
            telemetry.addLine("op mode failed");
            telemetry.update();
            throw new HeartBeatException();
        }
        if (runtime.seconds() > watchDogExpiration) {
            System.out.println("watch dog expired");
            telemetry.addLine("watch dog expired");
            telemetry.update();
            throw new HeartBeatException();
        }
    }

    public void resetDeadWheels() {
//        shootL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shootL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shootR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shootR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        intake.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    protected double getCurrentAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).firstAngle;

    }
}
