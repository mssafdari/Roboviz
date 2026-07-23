package com.kinematics;
import java.util.ArrayList;
import com.math.Vector6;
import com.math.Matrix;
import com.math.se3group;
import com.math.se3algebra;
import com.math.se3ops;

public class jacobianBuilder
{
	public static ArrayList<Vector6> JacobianBody(ArrayList<se3algebra> Blist, Matrix thetaList)
	{
		if (thetaList.getCols() != 1)
		{
			throw new IllegalArgumentException("thetalist should be vector");
		}
		ArrayList<Vector6> jacobianData = new ArrayList<Vector6>();

		// Start from the last joint
		// T represents: e^[-B_n]θ_n · e^[-B_{n-1}]θ_{n-1} · ... · e^[-B_{i+1}]θ_{i+1}
		se3group T = new se3group(Matrix.identity(4));

		// Iterate from LAST joint to FIRST
		for (int i = thetaList.getRows() - 1; i >= 0; i--)
		{
			// J_bi = Ad(T) * B_i
			Vector6 Bvec = se3algebra.se3ToVec(Blist.get(i));
			Vector6 Jb_i = new Vector6(se3group.adjoint(T).adj.multiply(Bvec));
			jacobianData.add(0, Jb_i);  // Add at beginning to maintain correct order

			// Update T for the next joint (i-1)
			// T = T * e^[-B_i]θ_i  (RIGHT multiplication! Not LEFT)
			se3algebra se3alg = new se3algebra(Matrix.scalarMulti(-thetaList.get(i, 0), Blist.get(i).matrix));
			se3group expB = se3ops.matrixExp6(se3alg).se3g;
			T.matrix = T.matrix.multiply(expB.matrix);
			//               ↑ RIGHT multiply
		}

		return jacobianData;
	}

	public static ArrayList<Vector6> JacobianSpace(ArrayList<se3algebra> Slist, Matrix thetaList)
	{
		if (thetaList.getCols() != 1)
		{
			throw new IllegalArgumentException("thetalist should be vector");
		}
		se3group se3g=new se3group(Matrix.identity(4));
	    ArrayList<Vector6> jacobianData = new ArrayList<Vector6>();
		for (int i=0;i < thetaList.getRows();i++)
		{
			jacobianData.add(new Vector6(se3group.adjoint(se3g).adj.multiply(se3algebra.se3ToVec(Slist.get(i)))));
			se3g.matrix = se3g.matrix.multiply(se3ops.matrixExp6(new se3algebra(Matrix.scalarMulti(thetaList.get(i, 0), Slist.get(i).matrix))).se3g.matrix);
		}
		return jacobianData;
	}
}
