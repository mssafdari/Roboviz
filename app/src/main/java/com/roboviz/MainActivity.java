package com.roboviz;

import android.app.Activity;
import android.os.Bundle;
import java.io.InputStream;
import java.io.IOException;
import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserFactory;
import org.xmlpull.v1.XmlPullParserException;
import android.widget.TextView;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Map;
import android.util.Log;
import android.widget.Button;
import android.view.View;
import com.tests.matrixTests;
import android.Manifest;
import android.os.Build;
import android.content.pm.PackageManager;
import android.annotation.NonNull;
import android.view.Gravity;
import com.tests.So3Test;
import com.tests.se3tests;
import java.util.Arrays;

public class MainActivity extends Activity {

    private TextView debug;
	
    private static final String TAG= "activity";
    private static final int REQUEST_EXTERNAL_STORAGE = 1;
    private static String[] PERMISSIONS_STORAGE = {
        Manifest.permission.READ_EXTERNAL_STORAGE,
        Manifest.permission.WRITE_EXTERNAL_STORAGE
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        matrixTests.init(this);
        debug = findViewById(R.id.myText);
		debug.setTextIsSelectable(true);
        Button myButton = findViewById(R.id.testButton);
        
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
                requestPermissions(PERMISSIONS_STORAGE, REQUEST_EXTERNAL_STORAGE);
            }
        }
        myButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                try {
                    matrixTests mT = new matrixTests();
                    So3Test so3T = new So3Test();
                    se3tests se3T = new se3tests();
                    String msg = mT.run() + so3T.run() + se3T.run();
                    debug.setGravity(Gravity.CENTER);
                    debug.setText(msg);
					
                    Log.d(TAG, "Test result: " + msg);
                } catch (Exception e) {
                    Log.e(TAG, "Error in test: " + e.getMessage());
                    debug.setText("Error: " + e.getMessage());
                    e.printStackTrace();
                }
            }
        });

        try {
            checkFile();
            InputStream is = getAssets().open("ur5.urdf.xml");
            Robot robot = new UrdfParser().parse(is);
            debug.setText(robot.toDebugString());
        } catch (Exception e) {
            debug.setText(e.getMessage());
        }
    }
    
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        if (requestCode == REQUEST_EXTERNAL_STORAGE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                Log.d(TAG, "Storage permission granted!");
            } else {
                Log.e(TAG, "Storage permission denied!");
            }
        }
    }
    private void checkFile(){
        try {
            // First check if file exists in assets
            String[] files = getAssets().list("");
            boolean fileFound = false;
            for (String file : files) {
                if (file.equals("ur5.urdf.xml")) {
                    fileFound = true;
                    break;
                }
            }

            if (!fileFound) {
                debug.setText("File not found in assets! Available files: " + Arrays.toString(files));
                return;
            }
        }catch (Exception e) {
            debug.setText(e.getMessage());
        }
    }
}
