package com.dynamics;
import com.math.Matrix;
import com.math.se3group;
import com.math.Vector6;
import java.util.ArrayList;
import com.math.rotPos;
import com.math.se3algebra;
import com.math.so3algebra;
import com.math.spatialInertia;
import com.math.omgVel;
import com.math.se3ops;
import com.kinematics.jacobianBuilder;
import com.math.Vector3;
import com.kinematics.dynConfig;
import android.animation.StateListAnimator;
import com.kinematics.forwardKinematics;
import java.time.temporal.Temporal;
import com.math.Vector;

public class dynamics
{
	private final static double g=-9.81;
	public static double dt=.05;
	public static double res=.05;
	private final static double[][] vdot0= new double[][]{{0},{0},{0},{0},{0},{g}};
	//adV = ad(V)
	public static Matrix lieBracket(se3algebra se3alg)
	{
		omgVel Og= new omgVel(se3alg.getOmega(), se3alg.getVelocity());
		double[][] lieBra= Matrix.zeros(6, 6).getData();
		Matrix.setSubMatrix(lieBra, 0, 0, Og.omg.matrix.getData());
		Matrix.setSubMatrix(lieBra, 3, 3, Og.omg.matrix.getData());
		Matrix.setSubMatrix(lieBra, 3, 0, so3algebra.vecToSo3(Og.vel).matrix.getData());
		return new Matrix(lieBra);
	}

	//taulist = InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip, Mlist,Glist,Slist)
	public static Vector inverseDynamics(velPos thetaDTheta, Vector ddthetaList, Vector6 Ftip,
										 robotModel model)
	{
		Matrix massM=massMatrix(thetaDTheta.getThetaList(), model);
		Vector VelQuadForces=velQuadraticForces(thetaDTheta.getThetaList(), thetaDTheta.getDThetaList(), model);
		Vector gforces=gravityForces(thetaDTheta.getThetaList(), model);
		Vector eeForces=endEffectorForces(thetaDTheta.getThetaList(), Ftip, model);
		Vector massThetadd=massM.multiply(thetaDTheta.getDThetaList());
		return  massThetadd.add(VelQuadForces).add(gforces).add(eeForces);
	}

	//M = MassMatrix(thetalist,Mlist,Glist,Slist)
	public static Matrix massMatrix(Vector thetaList, robotModel model)
	{
		dynConfig res=initializeMatrices(thetaList, model);
		Matrix part1= res.A6nx6n.transpose().multiply(res.Ltheta6nx6n.transpose());
		Matrix part2= res.G6nx6n.multiply(res.Ltheta6nx6n).multiply(res.A6nx6n);
		return part1.multiply(part2);
	}

	//c = VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist)
	public static Vector velQuadraticForces(Vector thetaList, Vector dthetalist, robotModel model)
	{
		dynConfig res=initializeMatrices(thetaList, model);
		Matrix adAthetadot=buildLieBraAthetadot(res.Alist, dthetalist);
		Vector twistStack=buildTwistStack(thetaList, model);
		Matrix adV =buildDiagonalLieBracketOfTwists(twistStack);
		Matrix Wtheta=buildWtheta(res.Ltheta6nx6n);
		Matrix part1= res.A6nx6n.transpose().multiply(res.Ltheta6nx6n.transpose());
		Matrix part2= res.G6nx6n.multiply(res.Ltheta6nx6n).multiply(adAthetadot).multiply(Wtheta);
		Matrix part3=adV.transpose().multiply(res.G6nx6n);
		Matrix part4=res.Ltheta6nx6n.multiply(res.A6nx6n).multiply(dthetalist);
		return new Vector( part1.multiply(part2.add(part3)).multiply(part4).getData());
	}

	//grav = GravityForces(thetalist,g,Mlist,Glist,Slist)
	public static Vector gravityForces(Vector thetaList, robotModel model)
	{
	    dynConfig res=initializeMatrices(thetaList, model);
		Matrix part1= res.A6nx6n.transpose().multiply(res.Ltheta6nx6n.transpose());
		Matrix Vdot_base=buildVdotBase(thetaList.getRows());
		Matrix part2= res.G6nx6n.multiply(res.Ltheta6nx6n).multiply(Vdot_base);
		return new Vector(part1.multiply(part2).getData());
	}

	public static dynConfig initializeMatrices(Vector thetaList, robotModel model)
	{
		ArrayList<se3algebra> Alist = buildAiList(model.Slist, model.Mlist);
		Matrix A6nx6n = buildA6nx6(Alist);
		ArrayList<se3group> Tlist = buildTlist(model.Slist, thetaList);
		Matrix Ltheta6nx6n= buildLTheta6nx6n(Tlist);
		Matrix G6nx6n = buildG6nx6n(model.Glist);
		return new dynConfig(Alist, A6nx6n, Ltheta6nx6n, G6nx6n);
	}

	//JTFtip = EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist)
	public static Vector endEffectorForces(Vector thetaList, Vector6 Ftip, robotModel model)
	{
		ArrayList<Vector6> jac=jacobianBuilder.JacobianSpace(model.Slist, thetaList);
		double[][] jacdata=new double[6][jac.size()];
		for (int i=0;i < jac.size();i++)
		{
			Matrix.setSubMatrix(jacdata, 0, i, jac.get(i).getData());
		}
		Matrix jacM=new Matrix(jacdata);
		return jacM.multiply(Ftip);
	}

	//ddthetalist = ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip, Mlist,Glist,Slist)
	public static Vector forwardDynamics(velPos thetaDTheta, Vector tauList, Vector6 Ftip, robotModel model)
	{
        ArrayList<Vector> Jbi=buildJbi(thetaDTheta.getThetaList(),model);
		Matrix massM=new Matrix(thetaDTheta.getThetaList().getRows()*6);
		for(int i=0;i<thetaDTheta.getThetaList().getRows();i++){
			massM.add(Jbi.get(i).transpose().multiply(model.Glist.get(i).matrix).multiply(Jbi.get(i)));
		}
		Vector hthetathetadot=inverseDynamics(thetaDTheta,Vector.zeros(tauList.getRows()),new Vector6(),model);
		Vector eeForces=endEffectorForces(thetaDTheta.getThetaList(),Ftip,model);
		Vector rhs=tauList.subtract(hthetathetadot).subtract(eeForces);
		return massM.inverse().multiply(rhs);
	}

	//[thetalistNext,dthetalistNext] = EulerStep(thetalist,dthetalist,ddthetalist,dt)
	public static velPos eulerStep(velPos thetaDTheta, Vector ddThetaList)
	{
		Vector thetaList=thetaDTheta.getThetaList();
		Vector dThetaList=thetaDTheta.getDThetaList();
		Vector thetaPlus1,dthetaPlus1;
		thetaPlus1 = thetaList.add(Vector.scalarMulti(dt, dThetaList));
		dthetaPlus1 = dThetaList.add(Vector.scalarMulti(dt, ddThetaList));
		return new velPos(thetaPlus1, dthetaPlus1);
	}

	//taumat = InverseDynamicsTrajectory(thetamat,dthetamat,ddthetamat, g,Ftipmat,Mlist,Glist,Slist)
	public static Matrix inverseDynamicsTrajectory(ArrayList<velPos> thetaDThetaMat, ArrayList<Vector> ddThetaMat, ArrayList<Vector6> FtipMat, robotModel model)
	{
		double[][] taumat=new double[ddThetaMat.get(0).getRows()][ddThetaMat.size()];
		Matrix tau_i=new Matrix(ddThetaMat.get(0).getRows(),1);
		for(int i=0;i<ddThetaMat.size();i++){
			tau_i=inverseDynamics(thetaDThetaMat.get(i),ddThetaMat.get(i),FtipMat.get(i),model);
			Matrix.setSubMatrix(taumat,0,i,tau_i.getData());
		}
		return new Matrix(taumat);
	} 

	//[thetamat,dthetamat] = ForwardDynamicsTrajectory(thetalist, dthetalist,taumat,g,Ftipmat,Mlist,Glist,Slist,dt,intRes)
	public static ArrayList<velPos> forwadDynamicsTrajectory(velPos thetaDTheta,ArrayList<Vector> tauMat, ArrayList<Vector6> Ftip, robotModel model)
	{
		//double[][] ddTheta=new double[tauMat.get(0).getRows()][tauMat.get(0).getCols()];
		ArrayList<velPos> thetaDThetaList=new ArrayList<velPos>();
		Vector ddTheta_i= new Vector(tauMat.get(0).getRows());
		for(int i=0;i<tauMat.get(0).getCols();i++){
			ddTheta_i=forwardDynamics(thetaDTheta,tauMat.get(i),Ftip.get(i),model);
			//Matrix.setSubMatrix(ddTheta,0,i,ddTheta_i.getData());
			thetaDTheta=eulerStep(thetaDTheta,ddTheta_i);
			thetaDThetaList.add(thetaDTheta);
		}
		return thetaDThetaList;
	}
	private static ArrayList<se3algebra> buildAiList(ArrayList<se3algebra> Slist, ArrayList<se3group> Mlist)
	{
		ArrayList<se3algebra> Alist= new ArrayList<se3algebra>();
		Matrix temp = Matrix.identity(4);
		Vector6 Svec;
		for (int i=0;i < Slist.size();i++)
		{
			temp = Mlist.get(i).matrix.inverse();
			Svec = se3algebra.se3ToVec(Slist.get(i));
			Alist.add(new se3algebra(se3group.adjoint(new se3group(temp)).multiply(Svec)));
		}
		return Alist;
	}
	private static Matrix buildA6nx6(ArrayList<se3algebra> Alist)
	{
		Matrix A6nx6= new Matrix(Alist.size() * 6, Alist.size());
		for (int i=0;i < Alist.size();i++)
		{
			for (int j=0;j < 6;j++)
			{
				A6nx6.set(j + 6 * i, i, Alist.get(i).matrix.get(j, 0));
			}
		}
		return A6nx6;
	}
	private static Matrix buildG6nx6n(ArrayList<spatialInertia> Glist)
	{
		Matrix G6nx6n= new Matrix(Glist.size() * 6, Glist.size() * 6);
		for (int i=0;i < Glist.size();i++)
		{
			for (int j=0;j < 6;j++)
			{
				for (int k=0;k < 6;k++)
				{
				    G6nx6n.set(j, k, Glist.get(i).getMatrix().get(j, k));
				}
			}
		}
		return G6nx6n;
	}
	private static Matrix buildLieBraAthetadot(ArrayList<se3algebra> Alist, Vector dthetaList)
	{
		Matrix LBAT = new Matrix(Alist.size() * 6, Alist.size() * 6);
		Matrix LBATi = new Matrix(Alist.size(), Alist.size());

		Vector6 Avec= new Vector6();
		se3algebra Ase3alg;
		for (int i=0;i < Alist.size();i++)
		{
			Avec = new Vector6(Matrix.scalarMulti(dthetaList.get(i, 0), Alist.get(i).matrix));
			Ase3alg = se3algebra.vecToSe3(Avec);
			LBATi = lieBracket(Ase3alg);
			for (int j=0;j < 6;j++)
			{
				for (int k=0;k < 6;k++)
				{
					LBAT.set(j, k, LBATi.get(j, k));
				}
			}
		}
		return LBAT;
	}

	private static ArrayList<se3group> buildTlist(ArrayList<se3algebra> Slist, Vector thetaList)
	{
		se3group T=new se3group(Matrix.identity(4));
		ArrayList<se3group> Tlist=new ArrayList<se3group>();
		for (int i=0;i < Slist.size();i++)
		{
			T = new se3group(se3ops.matrixExp6(new se3algebra(Matrix.scalarMulti(thetaList.get(i, 0), Slist.get(i).matrix))).matrix.multiply(T.matrix));
			Tlist.add(T);
		}
		return Tlist;
	}

	private static Vector buildTwistStack(Vector thetaList, robotModel model)
	{
		ArrayList<Vector> Jbi=buildJbi(thetaList,model);
		Vector6 JbiThetadot=new Vector6();
		double[][] twistStack= new double[6*thetaList.getRows()][1];
		for (int i=0;i < Jbi.size();i++)
		{
			JbiThetadot = new Vector6(Jbi.get(i).multiply(thetaList));
			Matrix.setSubMatrix(twistStack, i*6, 0, JbiThetadot.getData());
		}
		return new Vector(twistStack);
	}



	private static Matrix buildDiagonalLieBracketOfTwists(Vector twistStack)
	{
		double[][] diagTwist=new double[twistStack.getRows()][twistStack.getRows()];
		Matrix lieBra= new Matrix(6);
		Vector6 twist_i= new Vector6();
		int vec6size=6;
		for (int i=0;i < twistStack.getRows() / vec6size;i++)
		{
			twist_i = new Vector6(twistStack.getSubMatrix(i * vec6size, 0, vec6size, 1));
			lieBra = lieBracket(se3algebra.vecToSe3(twist_i));
			Matrix.setSubMatrix(diagTwist, i * vec6size, i * vec6size, lieBra.getData());
		}
		return new Matrix(diagTwist);
	}

	private static Matrix buildLTheta6nx6n(ArrayList<se3group> Tlist)
	{
		double[][] LTheta=new double[Tlist.size() * 6][Tlist.size() * 6];
		Matrix adj=Matrix.identity(6);
		for (int i=0;i < Tlist.size();i++)
		{
			Matrix.setSubMatrix(LTheta, i * 6, i * 6, Matrix.identity(6).getData());
			for (int j=0;j < i;j++)
			{
				adj = se3group.adjoint(Tlist.get(i));
				Matrix.setSubMatrix(LTheta, i * 6, j * 6, adj.getData());
			}
		}
		return new Matrix(LTheta);
	}
	public static Matrix buildWtheta(Matrix LTheta)
	{
		double[][] Wtheta=new double[LTheta.getRows()][LTheta.getRows()];
		for (int i=0;i < (LTheta.getRows() / 6 - 1);i++)
		{
			Matrix.setSubMatrix(Wtheta, i * 6, i * 6, LTheta.getSubMatrix((i + 1) * 6, i * 6, 6, 6).getData());
		}
		return new Matrix(Wtheta);
	}
	private static Vector buildVdotBase(int numOfLinks)
	{
		double[][] Vdotbase=new double[numOfLinks * 6][1];
		Matrix.setSubMatrix(Vdotbase, (numOfLinks - 1) * 6, 0, vdot0);
		return new Vector(Vdotbase);
	}
	private static ArrayList<Vector> buildJbi(Vector thetaList, robotModel model)
	{
		se3group Tsb=forwardKinematics.FKinSpace(model.Mlist.get(thetaList.getRows()), model.Slist, thetaList);
		ArrayList<se3algebra> Blist=new ArrayList<se3algebra>();
		for (int i=0;i < model.Slist.size();i++)
		{
			Blist.add(new se3algebra(se3group.adjoint(Tsb).multiply(model.Slist.get(i).matrix)));
		}
		ArrayList<Vector6> JacBody=jacobianBuilder.JacobianBody(Blist, thetaList);
		ArrayList<Vector> Jbi=new ArrayList<Vector>();
		double[][] temp=new double[6][6];
		for (int i=0;i < JacBody.size();i++)
		{
			for (int j=0;j < i;j++)
			{
				Matrix.setSubMatrix(temp, 0, i, JacBody.get(i).getData());
				Jbi.add(new Vector(temp));
			}
		}
		return Jbi;
	}
}
