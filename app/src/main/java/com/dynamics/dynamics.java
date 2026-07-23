 package com.dynamics;
import com.math.Matrix;
import com.math.se3group;
import com.math.Vector6;
import java.util.ArrayList;
import android.renderscript.Matrix2f;
import android.net.wifi.aware.PublishConfig;
import com.math.rotPos;
import com.math.se3algebra;
import com.math.so3algebra;
import com.math.Se3;
import com.math.spatialInertia;
import com.math.omgVel;
import com.math.se3ops;
import com.kinematics.jacobianBuilder;
import com.math.Vector3;

public class dynamics
{
	final static double g=9.81;
	public static double dt=.05;
	public static double res=.05;
	//adV = ad(V)
	public static Matrix lieBracket(se3algebra se3alg){
		omgVel Og= new omgVel(se3alg.getOmega(),se3alg.getVelocity());
		double[][] lieBra= Matrix.zeros(6,6).getData();
		Matrix.setSubMatrix(lieBra,0,0,Og.omg.matrix.getData());
		Matrix.setSubMatrix(lieBra,3,3,Og.omg.matrix.getData());
		Matrix.setSubMatrix(lieBra,3,0,so3algebra.vecToSo3(Og.vel).matrix.getData());
		return new Matrix(lieBra);
	}
	
	//taulist = InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip, Mlist,Glist,Slist)
	public static ArrayList<Vector6> inverseDynamics(velPos thetaDTheta,Matrix ddthetaList,Vector6 Ftip,
	robotModel model){
		return new ArrayList<Vector6>();
	}
	
	//M = MassMatrix(thetalist,Mlist,Glist,Slist)
	public static Matrix massMatrix(Matrix thetaList,robotModel model){
		return Matrix.identity(4);
	}
	
	//c = VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist)
	public static Matrix velQuadraticForces(Matrix thetaList,Matrix dthetalist,robotModel model){
		return Matrix.identity(4);
	}
	
	//grav = GravityForces(thetalist,g,Mlist,Glist,Slist)
	public static Matrix gravityForces(Matrix thetaList,robotModel model){
		return Matrix.identity(4);
	}
	
	//JTFtip = EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist)
	public static Matrix endEffectorForces(Matrix thetaList,Vector6 Ftip,robotModel model){
		return Matrix.identity(4);
	}
	
	//ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip, Mlist,Glist,Slist)
	public static Matrix forwardDynamics(velPos thetaDTheta,Matrix tauList,Vector6 Ftip,robotModel model){
		return Matrix.identity(4);
	}
	
	//[thetalistNext,dthetalistNext] = EulerStep(thetalist,dthetalist,ddthetalist,dt)
	public static velPos eulerStep(velPos thetaDTheta,Matrix ddThetaList){
		Matrix thetaList=thetaDTheta.getThetaList();
		Matrix dThetaList=thetaDTheta.getDThetaList();
		Matrix thetaPlus1,dthetaPlus1;
		thetaPlus1 = thetaList.add(Matrix.scalarMulti(dt,dThetaList));
		dthetaPlus1 = dThetaList.add(Matrix.scalarMulti(dt,ddThetaList));
		return new velPos(thetaPlus1,dthetaPlus1);
	}
	
	//taumat = InverseDynamicsTrajectory(thetamat,dthetamat,ddthetamat, g,Ftipmat,Mlist,Glist,Slist)
	public static Matrix inverseDynamicsTrajectory(ArrayList<velPos> thetaDThetaMat,ArrayList<Matrix> ddThetaMat,ArrayList<Vector6> FtipMat,robotModel model){
		return Matrix.identity(4);
	} 
	
	//[thetamat,dthetamat] = ForwardDynamicsTrajectory(thetalist, dthetalist,taumat,g,Ftipmat,Mlist,Glist,Slist,dt,intRes)
	public static velPos forwadDynamicsTrajectory(velPos thetaDTheta,Matrix tauMat,ArrayList<Vector6> Ftip,robotModel model){
		return new velPos(new Vector3(),new Vector3());
	}
	private static ArrayList<se3algebra> buildAiList(ArrayList<se3algebra> Slist,ArrayList<se3group> Mlist){
		ArrayList<se3algebra> Alist= new ArrayList<se3algebra>();
		Matrix temp = Matrix.identity(4);
		Vector6 Svec;
		for(int i=0;i<Slist.size();i++){
			temp=Mlist.get(i).matrix.inverse();
			Svec=se3algebra.se3ToVec(Slist.get(i));
			Alist.add(new se3algebra(se3group.adjoint(new se3group(temp)).adj.multiply(Svec)));
		}
		return Alist;
	}
	private static Matrix buildA6nx6(ArrayList<se3algebra> Alist){
		Matrix A6nx6= new Matrix(Alist.size()*6,Alist.size());
		for(int i=0;i<Alist.size();i++){
			for(int j=0;j<6;j++){
				A6nx6.set(j+6*i,i,Alist.get(i).matrix.get(j,0));
			}
		}
		return A6nx6;
	}
	private static Matrix buildG6nx6n(ArrayList<spatialInertia> Glist){
		Matrix G6nx6n= new Matrix(Glist.size()*6,Glist.size()*6);
		for(int i=0;i<Glist.size();i++){
			for(int j=0;j<6;j++){
				for(int k=0;k<6;k++){
				    G6nx6n.set(j,k,Glist.get(i).getMatrix().get(j,k));
				}
			}
		}
		return G6nx6n;
	}
	private static Matrix buildLieBraAthetadot(ArrayList<se3algebra> Alist,Matrix dthetaList){
		Matrix LBAT = new Matrix(Alist.size()*6,Alist.size()*6);
		Matrix LBATi = new Matrix(Alist.size(),Alist.size());
		
		Vector6 Avec= new Vector6();
		se3algebra Ase3alg;
		for(int i=0;i<Alist.size();i++){
			Avec = new Vector6(Matrix.scalarMulti(dthetaList.get(i,0),Alist.get(i).matrix));
			Ase3alg= se3algebra.vecToSe3(Avec);
			LBATi=lieBracket(Ase3alg);
			for(int j=0;j<6;j++){
				for(int k=0;k<6;k++){
					   LBAT.set(j,k,LBATi.get(j,k));
				}
			}
		}
		return LBAT;
	}
	
	private static ArrayList<se3group> buildTlist(ArrayList<Vector6> Slist,Matrix thetaList){
		se3algebra se3alg;
		se3group T=new se3group(Matrix.identity(4));
		ArrayList<se3group> Tlist=new ArrayList<se3group>();
		for(int i=0;i<Slist.size();i++){
			se3alg = se3algebra.vecToSe3( Slist.get(i));
			T=new se3group(se3ops.matrixExp6(new se3algebra(Matrix.scalarMulti(thetaList.get(i,0),se3alg.matrix))).se3g.matrix.multiply(T.matrix));
			Tlist.add(T);
		}
		return Tlist;
	}
	
	private static Matrix buildTwistStack(ArrayList<se3algebra> Blist, Matrix thetaList){
		ArrayList<Vector6> JacBody=jacobianBuilder.JacobianBody(Blist,thetaList);
		double[][] twistStack= new double[6*JacBody.size()][1];
		Vector6 JbiThetadot=new Vector6();
		for(int i=0;i<JacBody.size();i++){
			JbiThetadot = new Vector6(Matrix.scalarMulti(thetaList.get(i,0),JacBody.get(i)));
			Matrix.setSubMatrix(twistStack,i,0,JbiThetadot.getData());
		}
		return new Matrix(twistStack);
	}
	
	private static Matrix buildDiagonalLieBracketOfTwists(Matrix twistStack){
		double[][] diagTwist=new double[twistStack.getRows()][twistStack.getRows()];
		Matrix lieBra= new Matrix(6);
		Vector6 twist_i= new Vector6();
		int vec6size=6;
		for(int i=0;i<twistStack.getRows()/vec6size;i++){
			twist_i=new Vector6(twistStack.getSubMatrix(i*vec6size,0,vec6size,1));
			lieBra=lieBracket(se3algebra.vecToSe3(twist_i));
			Matrix.setSubMatrix(diagTwist,i*vec6size,i*vec6size,lieBra.getData());
		}
		return new Matrix(diagTwist);
	}
}
