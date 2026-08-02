package com.tests;
import com.math.Matrix;
import android.util.Log;
import java.io.*;
import android.icu.text.*;
import java.util.Date;
import android.widget.Button;
import android.view.*;
import android.app.Activity;
import com.roboviz.R;
import android.os.Environment;
import com.math.so3group;
import com.math.Vector3;
import com.math.so3algebra;
import com.math.axis3theta;
import com.math.so3ops;
import java.util.Locale;
import com.math.se3group;
import com.math.rotPos;
import com.math.Vector6;
import com.math.se3algebra;
import com.math.se3ops;  

public class se3tests extends BaseTest
{
    private static final String TAG = "se3 tests";
    private String errorReport="";
    @Override
    protected String getTestTag()
    {
        return TAG;
    }

    @Override
    protected String[] getTestNames()
    {
        return new String[]{
            "testIsTransformation",
            "testAdjoint",
            "testTransInverse",
            "testTransToRp",
            "testRpToTrans",
            "testSe3ToVec",
            "testvecToSe3",
            "testMatrixExp6",
            "testMatrixLog6"
        };
    }

    @Override
    protected boolean[] runTests()
    {
        return new boolean[]{
            testIsTransformation(),
            testAdjoint(),
            testTransInverse(),
            testTransToRp(),
            testRpToTrans(),
            testSe3ToVec(),
            testvecToSe3(),
            testMatrixExp6(),
            testMatrixLog6()
        };
    }

    @Override
    protected String getError()
    {
        return errorReport;
    }

    private boolean testIsTransformation()
    {
        try
        {
            double[][] transform = new double[][]
            {{1,0,0,1},{
                    0,1,0,2},{
                    0,0,1,3},{
                    0,0,0,1}};

            return se3group.isTransformation(new Matrix(transform));
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from rotInv\n";
            return false;
        }
    }

    private boolean testAdjoint()
    {
        try
        {
            double[][] transform = new double[][]
            {{1,0,0,1},{
                    0,1,0,2},{
                    0,0,1,3},{
                    0,0,0,1}};
            se3group se3g = new se3group(new Matrix(transform));
            Matrix adj=se3group.adjoint(se3g);
            double[][] desired = new double[][]
            {{1,0,0,0,0,0},{
                    0,1,0,0,0,0},{
                    0,0,1,0,0,0},{
                    0,-3,2,1,0,0},{
                    3,0,-1,0,1,0},{
                    -2,1,0,0,0,1}};
            return adj.isEqual(new Matrix(desired));

        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from vectoso3\n";
        }
        return false;
    }

    private boolean testTransInverse()
    {
        try
        {
            double[][] transform = new double[][]
            {{1,0,0,1},{
                    0,1,0,2},{
                    0,0,1,3},{
                    0,0,0,1}};
            double[][] desired = new double[][]
            {{1,0,0,-1},{
                    0,1,0,-2},{
                    0,0,1,-3},{
                    0,0,0,1}};
            se3group se3f= se3group.transInverse(new se3group(new Matrix(transform)));
            return se3f.matrix.isEqual(new Matrix(desired));
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from so3tovec\n";
        }
        return false;
    }

    private boolean testTransToRp()
    {
        try
        {
            double[][] transform = new double[][]
            {{1,0,0,1},{
                    0,1,0,2},{
                    0,0,1,3},{
                    0,0,0,1}};
            rotPos Rp=se3group.transToRp(new se3group(new Matrix(transform)));
            return Rp.rot.matrix.isEqual(Matrix.identity(3)) && Rp.pos.isEqual(new Vector3(1, 2, 3));
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from axisang3\n";
        }
        return false;
    }
    
    private boolean testRpToTrans()
    {
        try
        {
            double[][] transform = new double[][]
            {{1,0,0,1},{
                    0,1,0,2},{
                    0,0,1,3},{
                    0,0,0,1}};
            se3group se3g=se3group.RpToTrans(new so3group(Matrix.identity(3)), new Vector3(1, 2, 3));
            return se3g.matrix.isEqual(new Matrix(transform));

        }
        catch (Exception e)
        {
            errorReport += "\n" + e.getMessage() + " from matrixexp3\n";
            e.printStackTrace();
        }
        return false;
    }

    private boolean testSe3ToVec()
    {
        try
        {
            double[][] transform = new double[][]
            {{0,-3,2 ,0},{
                    3,0,-1,1},{
                    -2,1,0,0},
                    {0,0,0,0}};
            Vector6 vecse3= new Vector6(new Vector3(1,2,3),new Vector3(0,1,0));
            return se3algebra.se3ToVec(new se3algebra(new Matrix(transform))).isEqual(vecse3);
        }
        catch (Exception e)
        {
            errorReport += "\n" + e.getMessage() + " from matrixexp3\n";
            e.printStackTrace();
        }
        return false;
    }
    
    
    private boolean testvecToSe3()
    {
        try
        {
            double[][] transform = new double[][]
            {{0,-3,2 ,0},{
                    3,0,-1,1},{
                    -2,1,0,0},
                {0,0,0,0}};
            Vector6 vecse3= new Vector6(new Vector3(1,2,3),new Vector3(0,1,0));
            return se3algebra.vecToSe3(vecse3).matrix.isEqual(new Matrix(transform));
        
        }
        catch (Exception e)
        {
            errorReport += "\n" + e.getMessage() + " from matrixexp3\n";
            e.printStackTrace();
        }
        return false;
    }
    
    private boolean testMatrixExp6()
    {
        try
        {
            double pi = Math.PI;
            double[][] Tsb = new double[][]
            {{Math.cos(pi/6),-1.0*Math.sin(pi/6),0,1},{
              Math.sin(pi/6),Math.cos(pi/6),0,2},{
              0,0,1,0},
                {0,0,0,1}};
            double[][] Tsc = new double[][]
            {{Math.cos(pi/3),-1.0*Math.sin(pi/3),0,2},{
                    Math.sin(pi/3),Math.cos(pi/3),0,1},{
                    0,0,1,0},
                {0,0,0,1}};
            double[][] matrixExp = new double[][]
            {{0,-1.0,0,3.366025404},{
              1,0,0,-3.366025404},{
                    0,0,0,0},
                {0,0,0,0}};
                Matrix msb=new Matrix(Tsb);
                Matrix msc=new Matrix(Tsc);
                Matrix me = Matrix.scalarMulti(Math.PI/6,new Matrix( matrixExp));
                Matrix res = msc.multiply(msb.inverse());
                errorReport+="mInv="+res.toString()+"\n";
            se3group se3g= se3ops.matrixExp6(new se3algebra(me));
            
            errorReport +="me6res="+se3g.matrix.toString()+"\n";
                return se3g.matrix.isEqual(res);
            
        }
        catch (Exception e)
        {
            errorReport += "\n" + e.getMessage() + " from matrixexp3\n";
            e.printStackTrace();
        }
        return false;
    }
    
    private boolean testMatrixLog6()
    {
        try
        {
            double pi = Math.PI;
            double[][] Tsb = new double[][]
            {{Math.cos(pi/6),-1.0*Math.sin(pi/6),0,1},{
                    Math.sin(pi/6),Math.cos(pi/6),0,2},{
                    0,0,1,0},
                {0,0,0,1}};
            
            double[][] Tsc = new double[][]
            {
                {Math.cos(pi/3), -1.0*Math.sin(pi/3), 0, 2},
                {Math.sin(pi/3), Math.cos(pi/3), 0, 1},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            };
            double[][] matrixExp = new double[][]
            {{0,-1.0,0,3.366025404},{
                    1,0,0,-3.366025404},{
                    0,0,0,0},
                {0,0,0,0}};
            Matrix msb=new Matrix(Tsb);
            Matrix msc=new Matrix(Tsc);
            Matrix me = new Matrix(matrixExp);
            Matrix res = msc.multiply(msb.inverse());
            se3algebra se3alg = se3ops.matrixLog6(new se3group(res));
			//silencing
            return se3alg.matrix.isEqual(me);
            
        }
        catch (Exception e)
        {
            errorReport += "\n" + e.getMessage() + " from matrixexp3\n";
            e.printStackTrace();
        }
        return false;
    }};
