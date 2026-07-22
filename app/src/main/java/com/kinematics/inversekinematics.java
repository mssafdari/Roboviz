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

public class inversekinematics
{
	public static double eomg=0.000001;
	public static double ev=0.000001;
	public static int maxIterations=100;

	public static IKresult IKinBody(se3algebra[] Blist, se3group M, se3algebra T, Matrix thetaList0)
	{
		int iteration=0;
		ArrayList<Vector6> jb=jacobianBuilder.JacobianBody(Blist, thetaList0);
		Vector3 Wb= new Vector3();
		Vector3 Vb= new Vector3();
		Matrix thetaList,jbody;
		se3group Tsb,X;
		se3algebra bodyse3alg;
		do{
			Tsb=forwardKinematics.FKinBody(M, Blist, thetaList0);
			X = new se3group(Tsb.matrix.inverse().multiply(T.matrix));//x=Tsb\T
			bodyse3alg=se3ops.matrixLog6(X).se3alg;
			jbody=ArrayListToMatrix(jb);
			thetaList = thetaList0.add(jbody.pseudoInverse().multiply(se3algebra.se3ToVec(bodyse3alg)));
			Wb = bodyse3alg.getOmega();
			Vb = bodyse3alg.getVelocity();
			iteration++;
		}while((Wb.norm() > eomg || Vb.norm() > ev) && iteration<maxIterations);
		IKresult Result= new IKresult();
		Result.thetaList = thetaList;
		if (maxIterations > 0)
		{
			Result.success = true;
		}
		return Result;
	}
	//Vb=MatrixLog6(x);
	//Vs=VecTose3(Adjoint(Tsb)*se3ToVec(Vb));

	public static IKresult IKinSpace(se3algebra[] Slist, se3group M, se3algebra T, Matrix thetaList0)
	{
		int iteration =0;
		ArrayList<Vector6> js=jacobianBuilder.JacobianSpace(Slist, thetaList0);
		Vector3 Ws= new Vector3();
		Vector3 Vs= new Vector3();
		se3group Tsb,X;
		se3algebra bodyse3alg,spacese3alg;
		Matrix thetaList,jSpace;
		do{
			Tsb=forwardKinematics.FKinSpace(M, Slist, thetaList0);
			X = new se3group(Tsb.matrix.inverse().multiply(T.matrix));
			bodyse3alg=se3ops.matrixLog6(X).se3alg;
			spacese3alg=se3algebra.vecToSe3( new Vector6( se3group.adjoint(Tsb).adj.multiply(se3algebra.se3ToVec(bodyse3alg))));
			jSpace=ArrayListToMatrix(js);
			thetaList = thetaList0.add(jSpace.pseudoInverse().multiply(se3algebra.se3ToVec(spacese3alg)));
			Ws = bodyse3alg.getOmega();
			Vs = bodyse3alg.getVelocity();
			iteration++;
		}while((Ws.norm() > eomg || Vs.norm() > ev ) && iteration<maxIterations);
		IKresult Result= new IKresult();
		Result.thetaList = thetaList;
		if (maxIterations > 0)
		{
			Result.success = true;
		}
		return Result;
	}

	private static Matrix ArrayListToMatrix(ArrayList<Vector6> list)
	{
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
