package com.math;
import java.io.PushbackInputStream;

public class So3 extends Matrix
{
	double[][] data = new double[3][3];
	static Vector3 axis;
	static double tetha;
	public So3(double[][] Data)
	{
		super(Data);
		this.data = Data;
	}
	public static So3 vecToSo3(Vector3 vec3)//
	{
		double[][] so3data = {
			{0, vec3.z * -1, vec3.y},
			{vec3.z, 0, vec3.x * -1},
			{vec3.y * -1, vec3.x, 0}
		};
        return new So3(so3data);
	}
	public static Vector3 so3ToVec(So3 so3)//
	{
		return new Vector3(so3.data[2][1],so3.data[0][2],so3.data[1][0]);
	}
	
	public static void axisAng3(Vector6 exp3){
		So3.tetha = exp3.omega.norm();
		So3.axis = exp3.omega.normalize();
	}
	
	public static Matrix matrixExp3(So3 so3Matrix) {
		if (isRotation(so3Matrix)) {
			Vector3 omg = so3ToVec(so3Matrix);
			double theta = omg.norm();

			if (Math.abs(theta) < 1e-10) {
				return Matrix.identity(3);
			}

			Vector3 unitOmg = omg.normalize();
			So3 omega = vecToSo3(unitOmg);

			Matrix identity = Matrix.identity(3);
			Matrix term1 = Matrix.scalarMulti(Math.sin(theta), omega);
			Matrix term2 = Matrix.scalarMulti(1 - Math.cos(theta), omega.multiply(omega));
			return identity.add(term1).add(term2);
		}

		throw new IllegalArgumentException("Input is not a valid rotation matrix.");
	}
	
	public So3 matrixLog3(So3 R){
		if(isRotation(R)){
			if(R.isEqual(Matrix.identity(3))){
				tetha=0;
				axis= new Vector3(0,0,0);
			}
			else if(R.trace()==-1){
				So3 so3 = new So3(Matrix.identity(3).data);
				tetha = Math.PI;
				if(R.data[3][3]!=-1){
					Matrix temp= Matrix.scalarMulti(1/Math.sqrt(2*(1+R.data[2][2])),new Vector3(R.data[0][2],R.data[1][2],1+R.data[2][2]));
					so3 = vecToSo3(new Vector3(temp.data[0][0],temp.data[1][0],temp.data[2][0]));
				}
				else if(R.data[2][2]!=-1){
					Matrix temp= Matrix.scalarMulti(1/Math.sqrt(2*(1+R.data[1][1])),new Vector3(R.data[0][1],1+R.data[1][1],R.data[2][1]));
					so3 = vecToSo3(new Vector3(temp.data[0][0],temp.data[1][0],temp.data[2][0]));
				}
				else{
					Matrix temp= Matrix.scalarMulti(1/Math.sqrt(2*(1+R.data[0][0])),new Vector3(1+R.data[0][0],R.data[1][0],R.data[2][0]));
					so3 = vecToSo3(new Vector3(temp.data[0][0],temp.data[1][0],temp.data[2][0]));
				}
				return so3;
			}
		}
		throw new IllegalArgumentException("Input is not a valid rotation matrix.");
	}
	
	public static boolean isRotation(Matrix m)//
	{
		double[][] mdata = m.data;
		if (mdata.length == 3 && mdata[0].length == 3)
		{
			Matrix rRTranspose = m.multiply(m.transpose());
			if (isEqual(rRTranspose, Matrix.identity(3)))
			{
				return true;
			}
		}
		return false;
	}

	public static So3 rotInv(Matrix m)//
	{
		if (isRotation(m))
		{
			return new So3(m.transpose().data);
		}
		throw new IllegalArgumentException(
			"Matrix isnt a rotatiin matrix."
		); 
	}
}
