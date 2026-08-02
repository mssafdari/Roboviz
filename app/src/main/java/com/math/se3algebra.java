package com.math;
import java.util.ArrayList;

public class se3algebra
{
	public Matrix matrix;
	public se3algebra(Matrix mat){
		matrix=mat;
	}
    public se3algebra(Vector3 omega,Vector3 vel){
        matrix=se3algebra.vecToSe3(new Vector6(omega,vel)).matrix;
    }
	
	public se3algebra(Vector6 vec6){
        matrix=se3algebra.vecToSe3(vec6).matrix;
    }
	
	public static Vector6 se3ToVec(se3algebra se3)
	{
		so3algebra omg =new so3algebra(se3.matrix.getSubMatrix(0,0,3,3));
		Vector3 vel = new Vector3(se3.matrix.getSubMatrix(0,3,3,1));
		return new Vector6(so3algebra.so3ToVec(omg),vel);
	}
	
	public static se3algebra vecToSe3(Vector6 vec6)
	{
		se3algebra vecTo = new se3algebra(Matrix.identity(4));
		Matrix.setSubMatrix(vecTo.matrix.data, 0,0,so3algebra.vecToSo3(vec6.omega).matrix.data);
		Matrix.setSubMatrix(vecTo.matrix.data,0,3,vec6.velocity.data);
		Matrix.setSubMatrix(vecTo.matrix.data,3,0,Matrix.zeros(1,4).data);
		return vecTo;
	}
	
	public static ArrayList<se3algebra> doubleToSe3algebraArray(double[][] data){
		if(data.length != 6){
			throw new IllegalArgumentException("row size should be 6");
		}
		ArrayList<se3algebra> array=new ArrayList<se3algebra>();
		for(int i=0;i<data[0].length;i++){
			array.add(se3algebra.vecToSe3(Vector6.getColumnAsVec6(data,i)));
		}
		return array;
	}
	
	public Vector3 getVelocity(){
        return new Vector3(matrix.data[0][3],matrix.data[1][3],matrix.data[2][3]);
    }
    public Vector3 getOmega(){
        Matrix rot=matrix.getSubMatrix(0,0,3,3);
		so3algebra omg= new so3algebra(rot);
        return so3algebra.so3ToVec(omg);
    }
	
}
