package com.math;

public class so3group
{
	public Matrix matrix;
	public so3group(Matrix mat)
    {
		matrix = mat;
	}
	public static boolean isRotation(Matrix m)
    {
        double[][] mdata = m.data;
        if (mdata.length == 3 && mdata[0].length == 3)
        {
            Matrix rRTranspose = m.multiply(m.transpose());
            // Use tolerance for floating-point comparison
            Matrix identity = Matrix.identity(3);
            if (rRTranspose.isEqual(identity))
            { // Add tolerance parameter
                // Also check determinant is 1
                double det = m.determinant(); // Assuming determinant method exists
                return Math.abs(det - 1.0) < 1e-9;
            }
        }
        return false;
    }
	public static so3group rotInv(so3group m)
	{
		if (isRotation(m.matrix))
		{
			return new so3group(m.matrix.transpose());
		}
		throw new IllegalArgumentException(
			"Matrix isnt a rotation matrix."
		); 
	}

    public static so3group roll(double ang)
    {
        double[][] data= new double[][]{{
                1,0,0},
            {0,Math.cos(ang),-1.0 * Math.sin(ang)},
            {0,Math.sin(ang),Math.cos(ang)}};
        return new so3group(new Matrix(data));
    }

    public static so3group pitch(double ang)
    {
        double[][] data= new double[][]{{
                Math.cos(ang),0,Math.sin(ang)},
            {0,1,0},
            {-1 * Math.sin(ang),0,Math.cos(ang)}};
        return new so3group(new Matrix(data));
    }

    public static so3group yaw(double ang)
    {
        double[][] data= new double[][]{{
                Math.cos(ang),-1 * Math.sin(ang),0},
            {Math.sin(ang),Math.cos(ang),0},
            {0,0,1}};
        return new so3group(new Matrix(data));
    }
    static double c(double t)
    {
        return Math.cos(t);
    }
    static double s(double t)
    {
        return Math.sin(t);
    }
    public static so3group generalRotation(Vector3 omega, double t)
    {

        double w1=omega.x,w2=omega.y,w3=omega.z;
        // Helper to get (1 - cos(theta))
        double oneMinusCt = 1.0 - c(t);

        double[][] rotMatrix = {
            {
                c(t) + w1 * w1 * oneMinusCt, 
                w1 * w2 * oneMinusCt - w3 * s(t), 
                w1 * w3 * oneMinusCt + w2 * s(t)
            },
            {
                w1 * w2 * oneMinusCt + w3 * s(t), 
                c(t) + w2 * w2 * oneMinusCt, 
                w2 * w3 * oneMinusCt - w1 * s(t)
            },
            {
                w1 * w3 * oneMinusCt - w2 * s(t), 
                w2 * w3 * oneMinusCt + w1 * s(t), 
                c(t) + w3 * w3 * oneMinusCt
            }
        };
        return new so3group(new Matrix(rotMatrix));
    }
}
