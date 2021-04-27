package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
    //
    private SimpleMatrix x_k_1, P_k_1;
    //system paramters
    private SimpleMatrix F, G, R, Q, H, I, B, F_T, H_T;

    public KalmanFilter(SimpleMatrix f, SimpleMatrix g, SimpleMatrix r, SimpleMatrix q, SimpleMatrix h) {
        F = f;
        F_T = F.transpose();
        G = g;
        R = r;
        Q = q;
        H = h;
        H_T = H.transpose();
        I = SimpleMatrix.identity(F.numRows());
    }

    public void setInitalPostion(SimpleMatrix x_0, SimpleMatrix P_0, SimpleMatrix B) {
        x_k_1 = x_0;
        P_k_1 = P_0;
        this.B = B;
    }


    /**
     * @param z - observation
     * @return
     */
    public SimpleMatrix update(SimpleMatrix z, SimpleMatrix u) {
        SimpleMatrix x = F.mult(x_k_1).plus((B.mult(u)));
        SimpleMatrix P = F.mult(P_k_1).mult(F_T).plus(Q);

        SimpleMatrix S = H.mult(P).mult(H_T).plus(R);
        SimpleMatrix K = P.mult(H_T).mult(S.invert());

        SimpleMatrix y = z.minus(H.mult(x));

        SimpleMatrix x_k = x.plus(K.mult(y));
        SimpleMatrix P_k = I.minus(K.mult(H)).mult(P);

        this.x_k_1 = x_k;
        this.P_k_1 = P_k;

        return x_k;
    }
}
