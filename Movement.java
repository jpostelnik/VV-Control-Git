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
