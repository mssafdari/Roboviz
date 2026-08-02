package com.kinematics;
import com.math.se3group;
import com.math.se3algebra;
import com.math.Matrix;
import com.roboviz.Robot;
import com.math.se3ops;
import java.util.List;
import com.math.Vector6;
import java.util.ArrayList;
import com.math.Vector3;
import javax.xml.transform.Result;
import com.math.Vector;
import com.math.se3opslog;
import java.io.StringWriter;
import java.io.PrintWriter;
import com.roboviz.MainActivity;

public class inversekinematics
{
	public static double eomg=0.000001;
	public static double ev=0.000001;
	public static int maxIterations=10;
    private static IKresult Result= new IKresult();
	private static String el="\n";

	public static IKresult IKinBody(ArrayList<se3algebra> Blist, se3group M, se3group T, Vector thetaList0)
	{
		Result.success = false;
		int iteration=0;
		ArrayList<Vector6> jb;
		Vector3 Wb= new Vector3();
		Vector3 Vb= new Vector3();
		Vector thetaList;
		Matrix jbody;
		se3group Tsb,X;
		se3algebra bodyse3alg;
        se3algebra se3alg=new se3algebra(Matrix.zeros(4,4));
		thetaList = thetaList0;
        try
        {
		Result.log += thetaList.toString();
        
            do{
                jb = jacobianBuilder.JacobianBody(Blist, thetaList);
                Tsb = forwardKinematics.FKinBody(M, Blist, thetaList);
                Result.log+="Tsb="+el+Tsb.matrix.toString()+el;
                X = new se3group(Tsb.matrix.inverse().multiply(T.matrix));//x=Tsb\T
                Result.log+="Tsb-1*Tdes="+el+X.matrix.toString()+el;
                
                se3alg = se3ops.matrixLog6(X);
                bodyse3alg = se3alg;
                MainActivity.appendLog("bodyse3alg=" + el + bodyse3alg.matrix.toString() + el);
                
                jbody = ArrayListToMatrix(jb);
                thetaList = thetaList.add(jbody.pseudoInverse().multiply(se3algebra.se3ToVec(bodyse3alg)));
                Wb = bodyse3alg.getOmega();
                Vb = bodyse3alg.getVelocity();
                iteration++;
            }while((Wb.norm() > eomg || Vb.norm() > ev) && iteration < maxIterations);
            //Result.log+="wb.norm="+Wb.norm()+el+"vb.norm="+Vb.norm()+el;
            Result.thetaList = thetaList;
             /*for(int i=0;i<jb.size();i++){
             Result.log+="jb("+i+")="+el+jb.get(i).toString()+el;
             }
             Result.log+="jbody="+el+jbody.toString()+el;*/

            if (maxIterations > iteration)
            {
                Result.success = true;
            }
        }
        catch (Exception e)
        { 
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            String stackTrace = sw.toString();

            Result.log += "Exception: " + e.getMessage() + " from IKBody\n";
            Result.log += "Stack trace:\n" + stackTrace + "\n";
        }
        Result.log+=";)"+el;

		return Result;
	}
	//Vb=MatrixLog6(x);
	//Vs=VecTose3(Adjoint(Tsb)*se3ToVec(Vb));

	public static IKresult IKinSpace(ArrayList<se3algebra> Slist, se3group M, se3group T, Vector thetaList0)
	{
		Result.success = false;
		int iteration =0;
		ArrayList<Vector6> js;
		Vector3 Ws= new Vector3();
		Vector3 Vs= new Vector3();
		se3group Tsb,X;
		se3algebra bodyse3alg,spacese3alg;
		Vector thetaList;
		Matrix jSpace;
		thetaList = thetaList0;
		Result.log += thetaList.toString();
		do{
			js = jacobianBuilder.JacobianSpace(Slist, thetaList);
			Tsb = forwardKinematics.FKinSpace(M, Slist, thetaList);
			X = new se3group(Tsb.matrix.inverse().multiply(T.matrix));
			bodyse3alg = se3ops.matrixLog6(X);
			spacese3alg = se3algebra.vecToSe3(new Vector6(se3group.adjoint(Tsb).adj.multiply(se3algebra.se3ToVec(bodyse3alg))));
			jSpace = ArrayListToMatrix(js);
			thetaList = thetaList.add(jSpace.pseudoInverse().multiply(se3algebra.se3ToVec(spacese3alg)));
			Ws = spacese3alg.getOmega();
			Vs = spacese3alg.getVelocity();
			iteration++;
		}while((Ws.norm() > eomg || Vs.norm() > ev) && iteration < maxIterations);
		Result.log += "wb.norm=" + Ws.norm() + el + "vb.norm=" + Vs.norm() + el;

		Result.thetaList = thetaList;
		if (maxIterations > iteration)
		{
			Result.success = true;
		}
		return Result;
	}

	public static Matrix ArrayListToMatrix(ArrayList<Vector6> list)
	{
        if (list == null || list.isEmpty()) {
            Result.log += "Error: jb is null or empty\n";
            return new Matrix(3, 3); // Return identity or appropriate size
        }
		Matrix res = new Matrix(6, list.size());
		for (int i=0;i < res.getCols();i++)
		{
			for (int j=0;j < res.getRows();j++)
			{
				res.set(j, i, list.get(i).get(j, 0));
			}
		}
		return res;
	}
}
