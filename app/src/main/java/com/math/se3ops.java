package com.math;
import com.roboviz.Axis;
import com.roboviz.MainActivity;

public class se3ops
{
    private static String log;
    private static String el="\n";
	public static se3group matrixExp6(se3algebra se3alg, Boolean verbose)
	{
        log = "";
        MainActivity.appendTitle("entering MatrixExp6", verbose);
        se3group se3g;
        try
        {
            if (se3alg.matrix.getSubMatrix(0, 0, 3, 3).isSkewSymmetric())
            {
                if (se3alg.matrix.getSubMatrix(3, 0, 1, 4).isEqual(Matrix.zeros(4, 4).getSubMatrix(3, 0, 1, 4)))
                {
                    log += "input matrix=" + se3alg.matrix.toString() + "\n";
                    Vector6 V = se3algebra.se3ToVec(se3alg);
                    axis6theta at =Vector6.axisAng6(V);
                    Vector3 omg = at.axis.omega;
                    Vector3 vel = at.axis.velocity;
                    so3algebra omeg = so3algebra.vecToSo3(omg);
                    log += "theta=" + at.theta + "\n";
                    log += "omeg size=" + at.axis.velocity.rows + "x" + at.axis.velocity.cols + "\n";
                    log += "omg.norm=" + omg.norm() + "--- vel.norm=" + vel.norm() + "\n";
                    if (omg.norm() == 1)
                    {

                        so3group o = so3ops.matrixExp3(new so3algebra(Matrix.scalarMulti(at.theta, omeg.matrix)), false);
                        Matrix part1 = Matrix.scalarMulti(at.theta, Matrix.identity(3));
                        Matrix part2 = Matrix.scalarMulti(1.0 - Math.cos(at.theta), omeg.matrix);
                        Matrix part3 = Matrix.scalarMulti(at.theta - Math.sin(at.theta), omeg.matrix.multiply(omeg.matrix));
                        Matrix result = part1.add(part2).add(part3).multiply(at.axis.velocity);
                        log += "se3g_with_normOfOmega_1=" + result.rows + "x" + result.cols + "\n";
                        Matrix se3gmat = Matrix.blockToMatrix(new Matrix[][]{{o.matrix,result},{Matrix.zeros(1, 3),Matrix.identity(1)}});
                        se3g = new se3group(se3gmat);
                        log += "se3g_with_normOfOmega_1=" + se3g.matrix.toString();
                    }
                    else if (vel.norm() == 1.0 && omg.norm() == 0)
                    {
                        double[][] result = Matrix.identity(4).data;
                        Matrix.setSubMatrix(result, 0, 3, (Matrix.scalarMulti(at.theta, at.axis.velocity)).data);
                        se3g = new se3group(new Matrix(result));

                        log += "se3g_with_normOfVelocity_1=" + se3g.matrix.toString();
                    }
                    else
                    {
                        se3g = new se3group(Matrix.identity(4));
                        log += "se3g3=" + se3g.matrix.toString();
                    }
                    MainActivity.appendLog(log, verbose);
                    return se3g;
                }
            }
        }
        catch (Exception e)
        {
            MainActivity.appendLog(e.getMessage() + "from matrix exp6\n", verbose);
            throw new IllegalStateException("error in matrix exp6");
        }
		throw new IllegalArgumentException("Input is not a valid se3algebra matrix.");
	}
	public static se3algebra matrixLog6(se3group se3g, Boolean verbose)
	{
        log = "";
        MainActivity.appendTitle("entering MatrixLog6", verbose);
        se3algebra se3alg;
        try
        {
            if (se3group.isTransformation(se3g.matrix))
            {   

                Vector3 vel = new Vector3(se3g.matrix.data[0][3], se3g.matrix.data[1][3], se3g.matrix.data[2][3]);
                log += "vel=" + vel.toString();
                // Vector3 omgvec= new Vector3(se3gr.matrix.getSubMatrix(0, 0, 3, 3));
                if (se3g.matrix.getSubMatrix(0, 0, 3, 3).isEqual(Matrix.identity(3)))
                {
                    log += "rotation is identity in ML6\n";
                    so3algebra omega = so3algebra.vecToSo3(new Vector3());
                    double theta = vel.norm();
                    vel = vel.normalize();
                    double[][] m=new double[4][4];
                    Matrix.setSubMatrix(m, 0, 0, omega.matrix.data);
                    Matrix.setSubMatrix(m, 0, 3, vel.data);
                    Matrix mat=new Matrix(m);
                    mat = Matrix.scalarMulti(theta, mat);

                    log += "theta=" + theta;
                    se3alg = new se3algebra(mat);


                    log += "se3g=" + se3alg.matrix.toString();
                    MainActivity.appendLog(log, verbose);
                    return se3alg;
                }
                else
                {
                    log += "we have a rotation in ML6\n";
                    so3algebra so3alg = so3ops.matrixLog3((new so3group(se3g.matrix.getSubMatrix(0, 0, 3, 3))), false);
                    axis3theta a3t = Vector3.axisAng3(so3algebra.so3ToVec(so3alg));
                    log += "theta" + a3t.theta + "\n";
                    Matrix part1 =Matrix.scalarMulti(1.0 / a3t.theta, Matrix.identity(3));
                    Matrix ssOmg =so3algebra.vecToSo3(a3t.axis).matrix;
                    Matrix part2 = Matrix.scalarMulti(-.5, ssOmg);
                    Matrix part3 =Matrix.scalarMulti(1.0 / a3t.theta - 1.0 / (2.0 * Math.tan(a3t.theta / 2.0)), ssOmg.multiply(ssOmg));
                    Matrix result = part1.add(part2).add(part3).multiply(se3g.matrix.getSubMatrix(0, 3, 3, 1));
                    Matrix se3algmat = Matrix.blockToMatrix(new Matrix[][]{{so3alg.matrix,Matrix.scalarMulti(a3t.theta,result)},{Matrix.zeros(1, 3),Matrix.zeros(1, 1)}});

                    se3alg = new se3algebra(se3algmat);

                    log += "matrixlog6_res=" + se3alg.matrix.toString();
                    MainActivity.appendLog(log, verbose);
                    return se3alg;
                }
            }
		}
        catch (Exception e)
        {
            MainActivity.appendLog(e.getMessage() + "from matrixLog6\n", verbose);
            throw new IllegalStateException("error in matrix log6");
        }
		throw new IllegalArgumentException("Input is not a valid transformation matrix.");
	}
}
