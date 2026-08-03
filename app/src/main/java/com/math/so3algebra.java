package com.math;

public class so3algebra
{
	public Matrix matrix;
	
	public so3algebra(Matrix mat){
		matrix=mat;
	}
    public static boolean isSkewSymmetric(Matrix so3) {
        if (so3.data.length != 3 || so3.data[0].length != 3) {
            return false;
        }

        double epsilon = 1e-9; // Tolerance for floating-point errors

        // Check diagonal elements are zero
        for (int i = 0; i < 3; i++) {
            if (Math.abs(so3.data[i][i]) > epsilon) {
                return false;
            }
        }

        // Check skew-symmetric property: M[i][j] = -M[j][i]
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (Math.abs(so3.data[i][j] + so3.data[j][i]) > epsilon) {
                    return false;
                }
            }
        }

        return true;
    }
	public static so3algebra vecToSo3(Vector3 vec3)
	{
		double[][] so3data = {
			{0, vec3.z * -1.0, vec3.y},
			{vec3.z, 0, vec3.x * -1.0},
			{vec3.y * -1.0, vec3.x, 0}
		};
        return new so3algebra(new Matrix(so3data));
	}
	public static Vector3 so3ToVec(so3algebra so3)
	{
		return new Vector3(so3.matrix.data[2][1],so3.matrix.data[0][2],so3.matrix.data[1][0]);
	}
}
