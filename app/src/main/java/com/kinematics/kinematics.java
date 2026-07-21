package com.kinematics;
import com.math.se3group;
import com.math.se3algebra;
import com.math.Matrix;
import com.roboviz.Robot;
import com.math.se3ops;
import java.util.List;
import com.math.Vector6;
import java.util.ArrayList;

public class kinematics
{
    
    public kinematics(){
        
    }
    public static se3group FKinBody(se3group M,se3algebra[] Blist,double[] thetaList){
        se3group endEffectorT=M;
        for(int i=0;i<thetaList.length;i++){
            se3algebra se3alg = new se3algebra( Matrix.scalarMulti(thetaList[i], Blist[i].matrix));
            endEffectorT.matrix = endEffectorT.matrix.multiply(se3ops.matrixExp6(se3alg).se3alg.matrix);
            }
        return endEffectorT;
    }
    public static se3group FKinSpace(se3group M,se3algebra[] Slist,double[] thetaList){
        se3group endEffectorT=new se3group(Matrix.identity(4));
        for(int i=0;i<thetaList.length;i++){
            se3algebra se3alg = new se3algebra( Matrix.scalarMulti(thetaList[i], Slist[i].matrix));
            endEffectorT.matrix = endEffectorT.matrix.multiply((se3ops.matrixExp6(se3alg).se3alg.matrix));
        }
        endEffectorT.matrix=endEffectorT.matrix.multiply(M.matrix);
        return endEffectorT;
    }
	
	/*public static ArrayList<Vector6> JacobianBody(se3algebra[] Blist,double[] thetaList){
		
		se3group se3g=new se3group(Matrix.identity(4));
	    ArrayList<Vector6> jacobianData = new ArrayList<Vector6>();
		for(int i=thetaList.length;i>0;i--){
			jacobianData.add(new Vector6( se3group.adjoint(se3g).adj.multiply(se3algebra.se3ToVec(Blist[i]))));
			se3g.matrix=se3ops.matrixExp6(new se3algebra(Matrix.scalarMulti(-1.0*thetaList[i], Blist[i].matrix))).se3g.matrix.multiply(se3g.matrix);
		}
		return jacobianData;
	}*/
	
	public static ArrayList<Vector6> JacobianBody(se3algebra[] Blist, double[] thetaList) {
		ArrayList<Vector6> jacobianData = new ArrayList<Vector6>();

		// Start from the last joint
		// T represents: e^[-B_n]θ_n · e^[-B_{n-1}]θ_{n-1} · ... · e^[-B_{i+1}]θ_{i+1}
		se3group T = new se3group(Matrix.identity(4));

		// Iterate from LAST joint to FIRST
		for (int i = thetaList.length - 1; i >= 0; i--) {
			// J_bi = Ad(T) * B_i
			Vector6 Bvec = se3algebra.se3ToVec(Blist[i]);
			Vector6 Jb_i = new Vector6(se3group.adjoint(T).adj.multiply(Bvec));
			jacobianData.add(0, Jb_i);  // Add at beginning to maintain correct order

			// Update T for the next joint (i-1)
			// T = T * e^[-B_i]θ_i  (RIGHT multiplication! Not LEFT)
			se3algebra se3alg = new se3algebra(Matrix.scalarMulti(-thetaList[i], Blist[i].matrix));
			se3group expB = se3ops.matrixExp6(se3alg).se3g;
			T.matrix = T.matrix.multiply(expB.matrix);
			//               ↑ RIGHT multiply
		}

		return jacobianData;
	}
	
	public static ArrayList<Vector6> JacobianSpace(se3algebra[] Slist,double[] thetaList){
		se3group se3g=new se3group(Matrix.identity(4));
	    ArrayList<Vector6> jacobianData = new ArrayList<Vector6>();
		for(int i=0;i<thetaList.length;i++){
			jacobianData.add(new Vector6( se3group.adjoint(se3g).adj.multiply(se3algebra.se3ToVec(Slist[i]))));
			se3g.matrix=se3g.matrix.multiply(se3ops.matrixExp6(new se3algebra( Matrix.scalarMulti(thetaList[i], Slist[i].matrix))).se3g.matrix);
		}
		return jacobianData;
	}
	
}
