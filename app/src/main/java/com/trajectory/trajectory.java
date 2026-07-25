package com.trajectory;
import com.math.Matrix;
import com.math.se3group;
import com.math.Vector;
import java.util.ArrayList;
import com.math.se3ops;
import com.math.so3group;
import com.math.Vector3;
import com.math.so3ops;

public class trajectory
{
	public enum timeScalingMethod
	{
        CUBIC,
        QUINTIC
		}
	//s = CubicTimeScaling(Tf,t)
	public static double cubicTimeScaling(double Tf, double t)
	{
		double fraction=t / Tf;
		return 3.0 * fraction * fraction - 2.0 * Math.pow(fraction, 3);
	}

	//s = QuinticTimeScaling(Tf,t)
	public static double quinticTimeScaling(double Tf, double t)
	{
		double fraction=t / Tf;
		return 10.0 * Math.pow(fraction, 3) - 15.0 * Math.pow(fraction, 4) + 6.0 * Math.pow(fraction, 5);
	}

	//traj = JointTrajectory(thetastart,thetaend,Tf,N,method)
	public static ArrayList<Vector> jointTrajectory(Vector thetaStart, Vector thetaEnd, double Tf, double N, timeScalingMethod method)
	{
		double dt=Tf / (N - 1);
		ArrayList<Vector> Jtrajectory=new ArrayList<Vector>();
		Vector temp= new Vector(thetaStart.getRows());
		if (method == timeScalingMethod.CUBIC)
		{
			for (int i=0;i < N;i++)
			{
				temp = thetaStart.add(Vector.scalarMulti(cubicTimeScaling(Tf, i * dt), (thetaEnd.subtract(thetaStart))));
				Jtrajectory.add(temp);
			}
		}
		else
		{
			for (int i=0;i < N;i++)
			{
				temp = thetaStart.add(Vector.scalarMulti(quinticTimeScaling(Tf, i * dt), (thetaEnd.subtract(thetaStart))));
				Jtrajectory.add(temp);
			}
		}
		return Jtrajectory;
	}

	//traj = ScrewTrajectory(Xstart,Xend,Tf,N,method)
	public static ArrayList<se3group> screwTrajectory(se3group Xstart, se3group Xend, double Tf, double N, timeScalingMethod method)
	{
		double dt=Tf / (N - 1);
		ArrayList<se3group> Jtrajectory=new ArrayList<se3group>();
		se3group temp= new se3group(Matrix.identity(4));
		se3group XstartInv=new se3group(Xstart.matrix.inverse());
		if (method == timeScalingMethod.CUBIC)
			for (int i=0;i < N;i++)
			{
				temp= se3ops.matrixLog6(new se3group(XstartInv.matrix.multiply(Xend.matrix))).se3g;
				temp =new se3group(Xstart.matrix.multiply(Matrix.scalarMulti(cubicTimeScaling(Tf,i*dt),temp.matrix)));
				Jtrajectory.add(temp);
			}
			else{
				for (int i=0;i < N;i++)
				{
					temp= se3ops.matrixLog6(new se3group(XstartInv.matrix.multiply(Xend.matrix))).se3g;
					temp =new se3group(Xstart.matrix.multiply(Matrix.scalarMulti(quinticTimeScaling(Tf,i*dt),temp.matrix)));
					Jtrajectory.add(temp);
				}
			}
		return Jtrajectory;
	}

	//traj = CartesianTrajectory(Xstart,Xend,Tf,N,method)
	public static ArrayList<se3group> cartesianTrajectory(se3group Xstart, se3group Xend, double Tf, double N, timeScalingMethod method)
	{
		double dt=Tf / (N - 1);
		ArrayList<se3group> Jtrajectory=new ArrayList<se3group>();
		so3group rotStart= se3group.transToRp(Xstart).rot;
		Vector3 posStart= se3group.transToRp(Xstart).pos;
		so3group rotEnd= se3group.transToRp(Xend).rot;
		Vector3 posEnd= se3group.transToRp(Xend).pos;
		so3group tempRot= new so3group(Matrix.identity(3));
		Vector3 tempPos=new Vector3();
		se3group temp= new se3group(Matrix.identity(4));
		if(method==timeScalingMethod.CUBIC){
			for (int i=0;i < N;i++)
			{
				tempRot= so3ops.matrixLog3(new so3group(rotStart.matrix.transpose().multiply(rotEnd.matrix))).so3g;
				tempRot =new so3group(rotStart.matrix.multiply(Matrix.scalarMulti(cubicTimeScaling(Tf,i*dt),tempRot.matrix)));
				tempPos =new Vector3(posStart.add(Vector.scalarMulti(cubicTimeScaling(Tf, i * dt), (posEnd.subtract(posStart)))));
				temp=se3group.RpToTrans(tempRot,tempPos);
				Jtrajectory.add(temp);
			}
		}
		else{
			for (int i=0;i < N;i++)
			{
				tempRot= so3ops.matrixLog3(new so3group(rotStart.matrix.transpose().multiply(rotEnd.matrix))).so3g;
				tempRot =new so3group(rotStart.matrix.multiply(Matrix.scalarMulti(quinticTimeScaling(Tf,i*dt),tempRot.matrix)));
				tempPos =new Vector3(posStart.add(Vector.scalarMulti(cubicTimeScaling(Tf, i * dt), (posEnd.subtract(posStart)))));
				temp=se3group.RpToTrans(tempRot,tempPos);
				Jtrajectory.add(temp);
			}
		}
		return Jtrajectory;
	}
}
