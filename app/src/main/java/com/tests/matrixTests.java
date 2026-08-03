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
import java.security.Identity;
import com.roboviz.MainActivity;  

public class matrixTests extends BaseTest {
    private static final String TAG = "matrix test";
	private String errorReport="";
	private static final String el="\n";

    @Override
    protected String getTestTag() {
        return TAG;
    }

    @Override
    protected String[] getTestNames() {
        return new String[]{
            "testing matrix multiplication",
            "testing transpose",
            "testing inverse",
			"testPseudoInverse"
        };
    }

    @Override
    protected boolean[] runTests() {
        MainActivity.appendTitle("\n***Running Matrix tests***\n",true);
        return new boolean[]{
            testMatrixMultiply(),
            testTranspose(),
            testInverse(),
			testPseudoInverse()
        };
    }
    
    @Override
    protected String getError()
    {
        return errorReport;
    }

    // Individual test methods return boolean instead of String
    private boolean testMatrixMultiply() {
        Matrix a = new Matrix(new double[][]{{8, 2, 3, 4, 5}, {6, 7, 8, 9, 10},
								  {11, 12, 13, 14, 15}, {16, 17, 18, 19, 20}});

        Matrix b = new Matrix(new double[][]{{2, 5, 1, 3, 4, 2},
								  {7, 1, 8, 6, 9, 5}, {3, 9, 2, 7, 1, 8},
								  {6, 4, 5, 2, 3, 7}, {9, 8, 6, 4, 2, 1}});
        Matrix c = new Matrix(new double[][]{{108, 125, 80, 85, 75, 83},
								  {229, 225, 183, 174, 142, 184},
								  {364, 360, 293, 284, 237, 299},
								  {499, 495, 403, 394, 332, 414}});

        return c.isEqual(a.multiply(b));
    }

    private boolean testTranspose() {
        Matrix mat = Matrix.rand(4, 5);
        Matrix matTr = mat.transpose();
        boolean res = true;
        for (int i = 0; i < mat.getRows(); i++) {
            for (int j = 0; j < mat.getCols(); j++) {
                if (mat.get(i, j) != matTr.get(j, i)) {
                    res = false;
                }
            }
        }
        return res;
    }

    private boolean testInverse() {
        Matrix mat = Matrix.rand(5, 5);
        Matrix matInv = mat.inverse();
        Matrix matMatInv = mat.multiply(matInv);
        return matMatInv.isEqual(Matrix.identity(5));
    }
	
	private boolean testPseudoInverse(){
		try{
		Matrix mat=Matrix.rand(4,6);
		Matrix pseudo=mat.pseudoInverse();
	    errorReport+="mat="+el+mat.toString()+el;
			errorReport+="pseudo="+el+pseudo.toString()+el;
		Matrix identity= mat.multiply(mat.pseudoInverse());
		errorReport+="pseudo * mat="+el+identity.toString()+el;
		return identity.isEqual(Matrix.identity(4));	
		}
		catch (Exception e)
        {
            errorReport += e.getMessage() + "from FKinBody\n";
            return false;
        }
	}
}
