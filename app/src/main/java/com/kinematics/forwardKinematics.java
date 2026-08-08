package com.kinematics;
import com.math.se3group;
import com.math.se3algebra;
import com.math.Matrix;
import com.math.se3ops;
import java.util.ArrayList;
import com.math.Vector;
import com.roboviz.MainActivity;

public class forwardKinematics
{
	public static se3group FKinBody(se3group M, ArrayList<se3algebra> Blist, Vector thetaList,Boolean verbose)
	{
        MainActivity.appendLog("{{entering Fk-body}}",verbose);
        se3group endEffectorT = new se3group(new Matrix(M.matrix.getData()));
        for (int i=0;i < thetaList.getRows();i++)
		{
            se3algebra se3alg = new se3algebra(Matrix.scalarMulti(thetaList.get(i), Blist.get(i).matrix));
            endEffectorT.matrix = endEffectorT.matrix.multiply(se3ops.matrixExp6(se3alg,verbose).matrix);
		}
        MainActivity.appendLog("{{exiting Fk-body}}",verbose);
        return endEffectorT;
    }
    
    public static se3group FKinSpace(se3group M,ArrayList<se3algebra> Slist, Vector thetaList,Boolean verbose)
	{
        MainActivity.appendLog("{{entering Fk-space}}",verbose);
        se3group endEffectorT=new se3group(Matrix.identity(4));
        for (int i=0;i < thetaList.getRows();i++)
		{
            se3algebra se3alg = new se3algebra(Matrix.scalarMulti(thetaList.get(i), Slist.get(i).matrix));
            endEffectorT.matrix = endEffectorT.matrix.multiply((se3ops.matrixExp6(se3alg,verbose).matrix));
        }
        endEffectorT.matrix = endEffectorT.matrix.multiply(M.matrix);
        MainActivity.appendLog("{{exiting Fk-space}}",verbose);
        return endEffectorT;
    }
}
