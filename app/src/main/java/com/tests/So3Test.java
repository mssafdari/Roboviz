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
import com.roboviz.MainActivity;

public class So3Test extends BaseTest
{
    private static final String TAG = "matrix test";
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
            "testing rotInv",
			"testing Vectoso3",
			"testing so3toVec",
			"testing AxisAng3",
			"testing MatrixExp3",
			"testing MatrixLog3",
            "testRandomLogExp3",
            "testPiCaseLogExp3"
        };
    }

    @Override
    protected boolean[] runTests()
	{
        MainActivity.appendTitle("\n***Running so3 tests***\n",true);
        return new boolean[]{
            testRotInv(),
            testVecToso3(),
            testso3ToVec(),
			testAxisAng3(),
			testMatrixExp3(),
			testMatrixLog3(),
            testRandomLogExp3(),
            testPiCaseLogExp3()
        };
    }
    
    @Override
    protected String getError(){
        //return errorReport;
        return "";
    }

    private boolean testRotInv()
	{
        try
        {
            double[][] rotData = new double[][]
            {{2.0/3,  -1.0/3  , 2.0/3},{
                    2.0/3,   2.0/3,  -1.0/3},{
                    -1.0/3 ,  2.0/3   , 2.0/3}};
            Matrix rot =new Matrix(rotData);
            return  so3group.rotInv(new so3group(rot)).matrix.isEqual(rot.transpose());

        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from rotInv\n";
        }
        return false;
	}

	private boolean testVecToso3()
	{
        try
        {
            Vector3 v = new Vector3(1, 2, 3);
            double[][] ssData = new double[][]
            {{0,-3,2 },{
                    3,0,-1},{
                    -2,1,0}};
            Matrix ssMat =new Matrix(ssData);
            so3algebra so3alg = new so3algebra(ssMat);
            return so3algebra.vecToSo3(v).matrix.isEqual(so3alg.matrix);
        }
		catch (Exception e)
        {
            errorReport += e.getMessage() + "from vectoso3\n";
        }
        return false;
    }

	private boolean testso3ToVec()
	{
        try
        {
            Vector3 v = new Vector3(1, 2, 3);
            double[][] ssData = new double[][]
            {{0,-3,2 },{
                    3,0,-1},{
                    -2,1,0}};
            Matrix ssMat =new Matrix(ssData);
            so3algebra so3alg = new so3algebra(ssMat);
            return so3algebra.so3ToVec(so3alg).isEqual(v);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from so3tovec\n";
        }
        return false;
    }

	private boolean testAxisAng3()
	{
        try
        {
            Vector3 v = new Vector3(0.4, 0.6, 0.6928);
            v=v.normalize();
            Matrix vTheta = Matrix.scalarMulti(1.5, v);
            so3algebra so3alg =  so3algebra.vecToSo3(new Vector3( vTheta));
            axis3theta a3t = Vector3.axisAng3(so3algebra.so3ToVec(so3alg));
            if ((a3t.theta - 1.5)<.00001  && a3t.axis.isEqual(v))
            {
                return true;
            }
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from axisang3\n";
        }
        return false;
    }

	private boolean testMatrixExp3() {
        try {
            so3algebra so3alg = so3algebra.vecToSo3(new Vector3(0.0, .866, .5));
            so3alg.matrix=Matrix.scalarMulti(Math.PI/6,so3alg.matrix);
            double[][] data = new double[][]{
                {0.866031163, -0.250000512 ,0.433000887 },
                {0.250000512, 0.966506317 ,0.058011059 },
                {-0.433000887 ,0.058011059 ,0.899524846}
            };
            //errorReport+=so3alg.matrix.toString();
            so3group so3g = so3ops.matrixExp3(so3alg,false);
            /*for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    errorReport+=  String.format(Locale.US,"%.9f ", so3g.matrix.get(i,j));
                }
                errorReport += "\n";
            }*/
            //errorReport+= so3g.log;
            return so3g.matrix.isEqual(new Matrix(data));
        } catch (Exception e) {
            errorReport +="\n"+ e.getMessage() + " from matrixexp3\n";
            e.printStackTrace();
        }
        return false;
    }

	private boolean testMatrixLog3()
	{
        try
        {
            so3algebra truth = so3algebra.vecToSo3(new Vector3(0.0, .866, .5));
            truth.matrix=Matrix.scalarMulti(Math.PI/6,truth.matrix);
            double[][] data = new double[][]{
                {0.866031163, -0.250000512 ,0.433000887 },
                {0.250000512, 0.966506317 ,0.058011059 },
                {-0.433000887 ,0.058011059 ,0.899524846}};
            so3algebra so3alg = so3ops.matrixLog3((new so3group(new Matrix(data))),false);
            //errorReport+= so3g.log;
            return so3alg.matrix.isEqual(truth.matrix);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from matrixlog3\n";
        }
        return false;
    }
    
    private boolean testRandomLogExp3()
    {
        try
        {
            Vector3 omg=new Vector3(Matrix.rand(3,1));
            so3group so3g= so3group.generalRotation(omg.normalize(),Math.PI/7);
            so3algebra so3alg= so3ops.matrixLog3(so3g,false);
            so3group newSo3g=so3ops.matrixExp3(so3alg,false);
            return so3g.matrix.isEqual(newSo3g.matrix);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from matrixlogexp3\n";
        }
        return false;
    }
    
    private boolean testPiCaseLogExp3()
    {
        try
        {
            so3group so3g= so3group.roll(Math.PI);
            so3algebra so3alg= so3ops.matrixLog3(so3g,false);
            so3group newSo3g=so3ops.matrixExp3(so3alg,false);
            return so3g.matrix.isEqual(newSo3g.matrix);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from matrixlogexp3\n";
        }
        return false;
    }
}

  
