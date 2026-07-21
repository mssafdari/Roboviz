package com.math;

public class Vector3 extends Matrix
{
    public double x,y,z;

	public Vector3()
	{
		super(to2D(0, 0, 0));
	}

    public Vector3(double X, double Y, double Z)
	{
        super(to2D(X, Y, Z));
		x = X;
		y = Y;
		z = Z;
    }

    public Vector3 negate(){
        return new Vector3 (Matrix.scalarMulti(-1.0,this));
    }
    
	public Vector3(Matrix v)
	{
		super(v.data);
        if (v.hasSameDimensions(Matrix.zeros(3, 1)))
		{
			this.x = v.data[0][0];
			this.y = v.data[1][0];
			this.z = v.data[2][0];	
		}
		else
		{
			throw new IllegalArgumentException(
				"Matrices must have the same dimensions."
			);
		}
	}
	private static double[][] to2D(double xx, double yy, double zz)
	{
		double[][] data2d = new double[3][1];
		data2d[0][0] = xx;
		data2d[1][0] = yy;
		data2d[2][0] = zz;
		return data2d;
	}
	
	public static axis3theta axisAng3(Vector3 exp3){
		axis3theta a3= new axis3theta();
		a3.theta = exp3.norm();
		a3.axis = exp3.normalize();
		return a3;
	}

    public double dot(Vector3 other)
	{
		return (this.multiply(other.transpose())).data[0][0];
	}

    public Vector3 cross(Vector3 other)
	{
		double i,j,k;
		i = y * other.z - z * other.y;
	    j = (x * other.z - z * other.x) * -1;
		k = x * other.y - y * other.x;
		return new Vector3(i, j, k);
	} 

    public double norm()
	{ 
		return Math.sqrt(x * x + y * y + z * z);
	}

	public static double norm(Vector3 vec3)
	{ 
		return Math.sqrt(vec3.x * vec3.x + vec3.y * vec3.y + vec3.z * vec3.z);
	}

    public Vector3 normalize()
	{
		double mag = norm();
		return new Vector3(x / mag, y / mag, z / mag);
	}
}
