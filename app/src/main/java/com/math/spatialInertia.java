package com.math;

public class spatialInertia
{
	public Matrix matrix=new Matrix(6,6);
	double mass;
	Matrix momInertia;
	public spatialInertia(Matrix mIne,double m){
		mass=m;
		if(mIne.hasSameDimensions(matrix.identity(3))){
			momInertia=mIne;
		}
		else{
			throw new IllegalArgumentException("moment of inertia should be 3x3");
		}
		Matrix.setSubMatrix(matrix.data,0,0,mIne.data);
		Matrix.setSubMatrix(matrix.data,3,3,Matrix.scalarMulti(mass,matrix.identity(3)).data);
	}
	public Matrix getMatrix(){
		return matrix;
	}
}
