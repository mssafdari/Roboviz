package com.math;
import java.lang.annotation.Retention;

public class Se3 extends Matrix
{
	public static So3 rotR;
	public static Vector3 vecP;
	public static Vector6 axis;
	public static double tetha;
	double[][] data = new double[4][4];
	public Se3(double[][] Data)
	{
		super(Data);
		this.data = Data;
	}
	public Se3()
	{
		this(Matrix.identity(4).data);
	}
	
	public Matrix adjoint(){
		transToRp(this);
		So3 so3p = So3.vecToSo3(vecP);
		So3 adj11 = new So3(so3p.multiply(rotR).data);
		Matrix adj = Matrix.blockMatrix(new Matrix[][]{{rotR,Matrix.zeros(3,3)},{adj11,rotR}});
		return adj;
	}
	
	public static void axisAng(Vector6 expc6){
		tetha =expc6.omega.norm();
		if(tetha!=1){
			axis =new Vector6(Matrix.scalarMulti(1.0/tetha,expc6));
		}
		else{
			axis =new Vector6(new Vector3(0,0,0),new Vector3(expc6.velocity.x/tetha,
			expc6.velocity.y/tetha,expc6.velocity.z/tetha));
		}
	}
	
	public Se3 vecToSe3(Vector6 vec6)
	{
		Se3 vecTo = new Se3();
		Matrix.setSubMatrix(vecTo.data, 0,0,So3.vecToSo3(vec6.omega).data);
		Matrix.setSubMatrix(vecTo.data,0,3,vec6.velocity.data);
		Matrix.setSubMatrix(vecTo.data,3,0,Matrix.zeros(1,4).data);
		return vecTo;
	}
	
	public Vector6 screwToAxis(Vector3 q,Vector3 s,double h){
		Vector3 normalS = s.normalize();
		Vector3 vel = new Vector3(Matrix.scalarMulti(h,s).subtract( s.cross(q)));
		Vector6 axis = new Vector6(normalS,vel);
		return axis;
	}
	
	public Vector6 se3ToVec(Se3 se3)
	{
		So3 omg =new So3(se3.getSubMatrix(0,0,3,3).data);
		Vector3 vel = new Vector3(se3.getSubMatrix(0,3,3,1));
		return new Vector6(So3.so3ToVec(omg),vel);
	}
	public Se3 RpToTrans(So3 R, Vector3 p)
	{
		if (So3.isRotation(R))
		{
			Se3 homo = new Se3();
			for (int i=0;i < data.length - 1;i++)
			{
				for (int j=0;j < data.length - 1;j++)
				{
					homo.data[i][j] = R.data[i][j];
				}
				homo.data[i][2] = p.data[i][0];
			}
			homo.data[2][2] = 1;
			return homo;
		}
		throw new IllegalArgumentException("Input is not a valid rotation matrix.");
	}
	public static void transToRp(Se3 se3)
	{
		for (int i=0;i < 3;i++)
		{
			for (int j=0;j < 3;j++)
			{
				rotR.data[i][j] = se3.data[i][j];
			}
			vecP.data[i][0] = se3.data[i][2];
		}
		if(!So3.isRotation(rotR)){
			throw new IllegalArgumentException("Input is not a valid se3 matrix.");
		}
	}
	public Se3 transInverse(Se3 T){
		transToRp(T);
		Se3 tInverse = new Se3();
		Matrix.setSubMatrix(tInverse.data,0,0,rotR.transpose().data);
		Matrix.setSubMatrix(tInverse.data,0,3, Matrix.scalarMulti(-1,rotR.transpose().multiply(vecP)).data);
		Matrix.setSubMatrix(tInverse.data,3,0,Matrix.zeros(1,3).data);
		Matrix.setSubMatrix(tInverse.data,3,3,Matrix.ones(1,1).data);
		
		return tInverse;
	}
}
