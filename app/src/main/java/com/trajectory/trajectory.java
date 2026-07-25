package com.trajectory;
import com.math.Matrix;
import com.math.se3group;

public class trajectory
{
	public enum timeScalingMethod{
        CUBIC,
        QUINTIC
		}
	//s = CubicTimeScaling(Tf,t)
	public static double cubicTimeScaling(double Tf,double t){
		return 0;
	}
	
	//s = QuinticTimeScaling(Tf,t)
	public static double quinticTimeScaling(double Tf,double t){
		return 0;
	}
	
	//traj = JointTrajectory(thetastart,thetaend,Tf,N,method)
	public static Matrix jointTrajectory(Matrix thetaStart,Matrix thetaEnd,double Tf,double N,timeScalingMethod method){
		return Matrix.identity(6);
	}
	
	//traj = ScrewTrajectory(Xstart,Xend,Tf,N,method)
	public static Matrix screwTrajectory(se3group Xstart,se3group Xend,double Tf,double N,timeScalingMethod method){
		return Matrix.identity(6);
	}
	
	//traj = CartesianTrajectory(Xstart,Xend,Tf,N,method)
	public static Matrix cartesianTrajectory(se3group Xstart,se3group Xend,double Tf,double N,timeScalingMethod method){
		return Matrix.identity(6);
	}
}
