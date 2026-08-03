package com.math;
import com.roboviz.MainActivity;

public class so3ops
{
    private static String log;
	public static so3group matrixExp3(so3algebra so3alg,Boolean verbose)
    {
        log="";
        MainActivity.appendTitle("matrixExp3",verbose);
        
        so3group so3g;
        try
        {
            log+= so3alg.matrix.toString() + "\n";
            if (so3algebra.isSkewSymmetric(so3alg.matrix) || true)
            {
                Vector3 omg =so3algebra.so3ToVec(so3alg);
                double theta = omg.norm();

                if (Math.abs(theta) < 1e-10)
                {
                    so3g = new so3group(Matrix.identity(3));
                    MainActivity.appendLog( Matrix.identity(3).toString(),false);
                    return so3g;
                }

                Vector3 unitOmg = omg.normalize();
                so3algebra omega =so3algebra.vecToSo3(unitOmg);

                Matrix identity = Matrix.identity(3);
                Matrix term1 = Matrix.scalarMulti(Math.sin(theta), omega.matrix);
                Matrix term2 = Matrix.scalarMulti(1.0 - Math.cos(theta), omega.matrix.multiply(omega.matrix));
                so3g = new so3group(identity.add(term1).add(term2));
                log+= "theta="+ theta+"\n";
                log+= "omega-hat="+omega.matrix.toString()+"\n";
                log += "I=" + identity.toString() + "\n";
                log += "sin(theta)[omega]=" + term1.toString() + "\n";
                log += "1-cos(theta)[omega]^2=" + term2.toString() + "\n";
                log += "result="+identity.add(term1).add(term2)+"\n";
                MainActivity.appendLog(log,verbose);
                return so3g;
            }
        }
        catch (Exception e)
        {
            throw new IllegalStateException("error at matrixexp3");
        }

		throw new IllegalArgumentException("Input is not a valid skew symmetric matrix.");
	}
	public static so3algebra matrixLog3(so3group R,boolean verbose)
    {
        log="";
        if(verbose)
        MainActivity.appendTitle("matrixExp3",verbose);
        so3algebra so3alg;
        try
        {
            so3alg = new so3algebra(Matrix.identity(3));
            axis3theta at = new axis3theta();
            Matrix temp = Matrix.zeros(3,3);
            Vector3 Vec= new Vector3();
            
            if (so3group. isRotation(R.matrix))
            {
                if (R.matrix.isEqual(Matrix.identity(3)))
                {
                    MainActivity.appendLog("mexp3_identity",verbose);
                    return so3alg;
                }
                else if (R.matrix.trace() == -1)
                {
                    MainActivity.appendLog("mexp3_pi_trace=-1",verbose);
                    at.theta = Math.PI;
                    if (R.matrix.data[2][2] != -1)
                    {
                        temp = Matrix.scalarMulti(1 / Math.sqrt(2 * (1 + R.matrix.data[2][2])), new Vector3(R.matrix.data[0][2], R.matrix.data[1][2], 1 + R.matrix.data[2][2]));
                    }
                    else if (R.matrix.data[1][1] != -1)
                    {
                        temp = Matrix.scalarMulti(1 / Math.sqrt(2 * (1 + R.matrix.data[1][1])), new Vector3(R.matrix.data[0][1], 1 + R.matrix.data[1][1], R.matrix.data[2][1]));
                    }
                    else
                    {
                        temp = Matrix.scalarMulti(1 / Math.sqrt(2 * (1 + R.matrix.data[0][0])), new Vector3(1 + R.matrix.data[0][0], R.matrix.data[1][0], R.matrix.data[2][0]));
                    }
                    Vec = new Vector3(temp.data[0][0], temp.data[1][0], temp.data[2][0]);
                    so3alg = so3algebra.vecToSo3(Vec);
                    so3alg = new so3algebra(Matrix.scalarMulti(Math.PI,so3alg.matrix));// shoud i multiply by pi here
                    log += "vec=" + Vec.toString()+"\n";
                    log += "so3alg=" + so3alg.matrix.toString() + "\n";
                    MainActivity.appendLog(log,verbose);
                    return so3alg;
                }
                else
                {
                    MainActivity.appendLog("mexp3_theta-notZeroOrPi",verbose);
                    at.theta = Math.acos(.5 * (R.matrix.trace() - 1));
                    Matrix omega = Matrix.scalarMulti(.5 * at.theta / Math.sin(at.theta), R.matrix.subtract(R.matrix.transpose()));
                    Vec = so3algebra.so3ToVec(new so3algebra(omega));
                    so3alg = so3algebra.vecToSo3(Vec);  
                    log += "vec=" + Vec.toString()+"\n";
                    log += "so3alg=" + so3alg.matrix.toString() + "\n";
                    if(verbose)
                    MainActivity.appendLog(log,verbose);
                    return so3alg;
                }
            }
        }
        catch (Exception e)
        {
            throw new IllegalStateException("error in matrix log3");
        }
		throw new IllegalArgumentException("oops.");
	}
}
