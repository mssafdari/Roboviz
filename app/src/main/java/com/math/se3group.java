package com.math;
import com.roboviz.MainActivity;


public class se3group
{
	public Matrix matrix;
    private static String log;
	public se3group(Matrix mat){
		matrix=mat;
	}
	
	public static boolean isTransformation(Matrix m)
	{
		so3group R =new so3group( m.getSubMatrix(0,0,3,3));
		if(so3group.isRotation(R.matrix)){
			if(m.getSubMatrix(3,0,1,4).isEqual(Matrix.identity(4).getSubMatrix(3,0,1,4))){
				return true;
			}
		}
		return false;
	}
	
	public static Matrix adjoint(se3group T){
		rotPos Rp = transToRp(T);
        se3group adjoi =new se3group(Matrix.identity(4));
        log +="T=\n"+T.matrix.toString();
        log +="Rp.R=\n"+Rp.rot.matrix.toString();
        
        Vector3 vec = new Vector3(T.matrix.data[0][3],T.matrix.data[1][3],T.matrix.data[2][3]);
		so3algebra so3p = so3algebra.vecToSo3(vec);
        log +="vec=\n"+vec.toString();
        log +="Rp.p=\n"+Rp.pos.toString();
        log +="[p]=\n"+so3p.matrix.toString();
		so3algebra adj11 = new so3algebra(so3p.matrix.multiply(Rp.rot.matrix));
		Matrix adj = Matrix.blockToMatrix(new Matrix[][]{
                                              {Rp.rot.matrix, Matrix.zeros(3, 3)},
                                              {adj11.matrix, Rp.rot.matrix}
                                          });
        log +="adh=\n"+adj.toString();
        MainActivity.appendLog(log);
		return adj;
	}
	
	public static rotPos transToRp(se3group se3)
	{
		rotPos Rp = new rotPos();
		for (int i=0;i < 3;i++)
		{
			for (int j=0;j < 3;j++)
			{
				Rp.rot.matrix.data[i][j] = se3.matrix.data[i][j];
			}
			Rp.pos.data[i][0] = se3.matrix.data[i][3];
		}
		if(!so3group.isRotation(Rp.rot.matrix)){
			throw new IllegalArgumentException("Input is not a valid se3group matrix.");
		}
		return Rp;
	}
	
	public static se3group RpToTrans(so3group R, Vector3 p)
	{
		if (so3group.isRotation(R.matrix))
		{
			se3group homo = new se3group(Matrix.identity(4));
			for (int i=0;i <3;i++)
			{
				for (int j=0;j < 3;j++)
				{
					homo.matrix.data[i][j] = R.matrix.data[i][j];
				}
				homo.matrix.data[i][3] = p.data[i][0];
			}
			return homo;
		}
		throw new IllegalArgumentException("Input is not a valid rotation matrix.");
	}
	
	public static se3group transInverse(se3group T){
		rotPos Rp = transToRp(T);
		se3group tInverse = new se3group(Matrix.identity(4));
		Matrix.setSubMatrix(tInverse.matrix.data,0,0,Rp.rot.matrix.transpose().data);
		Matrix.setSubMatrix(tInverse.matrix.data,0,3, Matrix.scalarMulti(-1,Rp.rot.matrix.transpose().multiply(Rp.pos)).data);
		Matrix.setSubMatrix(tInverse.matrix.data,3,0,Matrix.zeros(1,3).data);
		Matrix.setSubMatrix(tInverse.matrix.data,3,3,Matrix.ones(1,1).data);

		return tInverse;
	}
    public Vector3 getPosition(){
        return new Vector3(matrix.data[0][3],matrix.data[1][3],matrix.data[2][3]);
    }
    public so3group getRotation(){
        Matrix rot=matrix.getSubMatrix(0,0,3,3);
        return new so3group(rot);
    }
}
