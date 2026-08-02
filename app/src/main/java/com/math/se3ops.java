package com.math;
import com.roboviz.Axis;
import com.roboviz.MainActivity;

public class se3ops
{
	public static se3group matrixExp6(se3algebra se3alg)
	{
        se3group se3g;
        try{
		if (se3alg.matrix.getSubMatrix(0, 0, 3, 3).isSkewSymmetric())
		{
			if (se3alg.matrix.getSubMatrix(3, 0, 1, 4).isEqual(Matrix.zeros(4, 4).getSubMatrix(3, 0, 1, 4)))
			{
                MainActivity.appendLog("input matrix="+se3alg.matrix.toString()+"\n");
				Vector6 V = se3algebra.se3ToVec(se3alg);
				axis6theta at =Vector6.axisAng6(V);
				Vector3 omg = at.axis.omega;
				Vector3 vel = at.axis.velocity;
				so3algebra omeg = so3algebra.vecToSo3(omg);
                MainActivity.appendLog("theta="+at.theta+"\n");
                MainActivity.appendLog("omeg size="+at.axis.velocity.rows+"x"+at.axis.velocity.cols+"\n");
                MainActivity.appendLog("omg.norm="+omg.norm()+"--- vel.norm="+vel.norm()+"\n");
				if (omg.norm() == 1)
				{
                    
					so3opslog o = so3ops.matrixExp3(new so3algebra(Matrix.scalarMulti(at.theta, omeg.matrix)));
					Matrix part1 = Matrix.scalarMulti(at.theta, Matrix.identity(3));
					Matrix part2 = Matrix.scalarMulti(1.0 - Math.cos(at.theta), omeg.matrix);
					Matrix part3 = Matrix.scalarMulti(at.theta - Math.sin(at.theta), omeg.matrix.multiply(omeg.matrix));
					Matrix result = part1.add(part2).add(part3).multiply(at.axis.velocity);
					MainActivity.appendLog("result1st="+result.rows+"x"+result.cols+"\n");
                    double[][] t1=new double[][]{{1,2,3},{4,5,6},{7,8,9}};
                    double[][] t2=new double[][]{{7},{4},{1}};
                    double[][] t3=new double[][]{{1,2,4}};
                    double[][] t4=new double[][]{{1}};
                    Matrix se3gmat = Matrix.blockToMatrix(new Matrix[][]{{o.so3g.matrix,result},{Matrix.zeros(1, 3),Matrix.identity(1)}});
					Matrix test = Matrix.blockToMatrix(new Matrix[][]{{new Matrix(t1),new Matrix(t2)},{new Matrix(t3),new Matrix(t4)}});
					
                    se3g= new se3group(se3gmat);
                   
                    MainActivity.appendLog("se3g1="+test.toString());
                    
				}
				else if (vel.norm() == 1.0 && omg.norm() == 0)
				{
					double[][] result = Matrix.identity(4).data;
					Matrix.setSubMatrix(result, 0, 3, (Matrix.scalarMulti(at.theta, at.axis.velocity)).data);
					se3g = new se3group(new Matrix(result));
                   
                    MainActivity.appendLog("se3g2="+se3g.matrix.toString());
				}
				else
				{
					se3g = new se3group(Matrix.identity(4));
                    MainActivity.appendLog("se3g3="+se3g.matrix.toString());
				}
                return se3g;
			}
		}
        }catch (Exception e)
        {
            MainActivity.appendLog( e.getMessage() + "from rotInv\n");
            throw new IllegalStateException("error ib matrix exp6");
        }
		throw new IllegalArgumentException("Input is not a valid se3algebra matrix.");
	}
	public static se3algebra matrixLog6(se3group se3gr)
	{
        se3algebra se3alg;
        try{
		if (se3group.isTransformation(se3gr.matrix))
		{   
            
			Vector3 vel = new Vector3(se3gr.matrix.data[0][3], se3gr.matrix.data[1][3], se3gr.matrix.data[2][3]);
            MainActivity.appendLog("vel="+vel.toString());
           // Vector3 omgvec= new Vector3(se3gr.matrix.getSubMatrix(0, 0, 3, 3));
			if (se3gr.matrix.getSubMatrix(0, 0, 3, 3).isEqual(Matrix.identity(3)))
			{
                MainActivity.appendLog("+xxxx+");
				so3algebra omega = so3algebra.vecToSo3(new Vector3());
				double theta = vel.norm();
				vel = vel.normalize();
				double[][] m=new double[4][4];
				Matrix.setSubMatrix(m, 0, 0, omega.matrix.data);
				Matrix.setSubMatrix(m, 0, 3, vel.data);
                Matrix mat=new Matrix(m);
				mat = Matrix.scalarMulti(theta, mat);
                MainActivity.appendLog("se3g="+mat.toString());
                
                MainActivity.appendLog("theta="+theta);
                se3alg =new se3algebra(mat);
	
                
                MainActivity.appendLog("se3g="+se3alg.matrix.toString());
                return se3alg;
			}
			else
			{
                MainActivity.appendLog("+++++");
				so3opslog so3alg = so3ops.matrixLog3((new so3group(se3gr.matrix.getSubMatrix(0, 0, 3, 3))));
				axis3theta a3t = Vector3.axisAng3(so3algebra.so3ToVec(so3alg.so3alg));
                MainActivity.appendLog("theta"+a3t.theta+"\n");
				Matrix part1 =Matrix.scalarMulti(1.0 / a3t.theta, Matrix.identity(3));
				Matrix ssOmg =so3algebra.vecToSo3(a3t.axis).matrix;
				Matrix part2 = Matrix.scalarMulti(-.5, ssOmg);
				Matrix part3 =Matrix.scalarMulti(1.0 / a3t.theta - 1.0 / (2.0 * Math.tan(a3t.theta / 2.0)), ssOmg.multiply(ssOmg));
				Matrix result = part1.add(part2).add(part3).multiply(se3gr.matrix.getSubMatrix(0, 3, 3, 1));
                Matrix se3algmat = Matrix.blockToMatrix(new Matrix[][]{{Matrix.scalarMulti(1.0/a3t.theta,so3alg.so3alg.matrix),result},{Matrix.zeros(1, 3),Matrix.zeros(1,1)}});
             
				se3alg =new se3algebra(se3algmat);
            
                MainActivity.appendLog("matrixlogres="+se3alg.matrix.toString());
                return se3alg;
			}
            }
		}catch (Exception e)
        {
            MainActivity.appendLog( e.getMessage() + "from rotInv\n");
            throw new IllegalStateException("error in matrix log6");
        }
		throw new IllegalArgumentException("Input is not a valid transformation matrix.");
	}
}
