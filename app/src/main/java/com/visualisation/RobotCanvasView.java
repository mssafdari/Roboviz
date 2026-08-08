package com.visualisation;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;
import java.util.ArrayList;
import com.math.se3group;
import com.math.Vector3;

public class RobotCanvasView extends View {
    private Paint jointPaint;
    private Paint linkPaint;
    private Paint textPaint;
    private ArrayList<se3group> jointPoses = new ArrayList<>();
    private float scale = 100f;  // Pixels per meter
    private float centerX = 500f;
    private float centerY = 800f;
    
    // Isometric projection angles
    private static final double ISO_ANGLE_X = Math.toRadians(30);
    private static final double ISO_ANGLE_Z = Math.toRadians(45);

    public RobotCanvasView(Context context) {
        super(context);
        init();
    }

    public RobotCanvasView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        jointPaint = new Paint();
        jointPaint.setColor(Color.RED);
        jointPaint.setStyle(Paint.Style.FILL);
        jointPaint.setAntiAlias(true);

        linkPaint = new Paint();
        linkPaint.setColor(Color.BLUE);
        linkPaint.setStyle(Paint.Style.STROKE);
        linkPaint.setStrokeWidth(6f);
        linkPaint.setAntiAlias(true);

        textPaint = new Paint();
        textPaint.setColor(Color.BLACK);
        textPaint.setTextSize(30f);
        textPaint.setAntiAlias(true);
    }

    public void setJointPoses(ArrayList<se3group> poses) {
        this.jointPoses = poses;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        canvas.drawColor(Color.WHITE);

        if (jointPoses == null || jointPoses.isEmpty()) {
            canvas.drawText("No robot data", 100, 200, textPaint);
            return;
        }

        // Draw coordinate axes
        drawAxes(canvas);

        // Draw links and joints
        float prevX = centerX;
        float prevY = centerY;
        
        for (int i = 0; i < jointPoses.size(); i++) {
            se3group pose = jointPoses.get(i);
            if (pose == null) continue;
            
            Vector3 position = pose.getPosition();
            float[] screenPos = projectToIsometric(position);
            
            float x = screenPos[0] + centerX;
            float y = screenPos[1] + centerY;
            
            // Draw link from previous joint
            if (i > 0) {
                canvas.drawLine(prevX, prevY, x, y, linkPaint);
            } else {
                // Draw base link
                canvas.drawLine(centerX, centerY, x, y, linkPaint);
            }
            
            // Draw joint (sphere)
            canvas.drawCircle(x, y, 20f, jointPaint);
            
            // Draw joint label
            String label = "J" + i;
            canvas.drawText(label, x + 10, y - 10, textPaint);
            
            prevX = x;
            prevY = y;
        }
        
        // Draw end effector if available
        if (!jointPoses.isEmpty()) {
            se3group lastPose = jointPoses.get(jointPoses.size() - 1);
            if (lastPose != null) {
                Vector3 endPos = lastPose.getPosition();
                float[] screenPos = projectToIsometric(endPos);
                float endX = screenPos[0] + centerX;
                float endY = screenPos[1] + centerY;
                
                Paint endPaint = new Paint();
                endPaint.setColor(Color.GREEN);
                endPaint.setStyle(Paint.Style.FILL);
                canvas.drawCircle(endX, endY, 25f, endPaint);
                canvas.drawText("End Effector", endX + 10, endY + 10, textPaint);
            }
        }
    }

    private float[] projectToIsometric(Vector3 pos) {
        double x = pos.x * scale;
        double y = pos.y * scale;
        double z = pos.z * scale;
        
        // Isometric projection matrix
        float screenX = (float)((x * Math.cos(ISO_ANGLE_Z) - y * Math.sin(ISO_ANGLE_Z)) * Math.cos(ISO_ANGLE_X));
        float screenY = (float)(-(x * Math.sin(ISO_ANGLE_Z) + y * Math.cos(ISO_ANGLE_Z)) * Math.sin(ISO_ANGLE_X) - z);
        
        return new float[]{screenX, screenY};
    }

    private void drawAxes(Canvas canvas) {
        Paint axisPaint = new Paint();
        axisPaint.setStrokeWidth(3f);
        axisPaint.setAntiAlias(true);
        
        // X axis (red)
        axisPaint.setColor(Color.RED);
        float[] xEnd = projectToIsometric(new Vector3(0.5f, 0, 0));
        canvas.drawLine(centerX, centerY, centerX + xEnd[0], centerY + xEnd[1], axisPaint);
        canvas.drawText("X", centerX + xEnd[0], centerY + xEnd[1] - 10, textPaint);
        
        // Y axis (green)
        axisPaint.setColor(Color.GREEN);
        float[] yEnd = projectToIsometric(new Vector3(0, 0.5f, 0));
        canvas.drawLine(centerX, centerY, centerX + yEnd[0], centerY + yEnd[1], axisPaint);
        canvas.drawText("Y", centerX + yEnd[0], centerY + yEnd[1] - 10, textPaint);
        
        // Z axis (blue)
        axisPaint.setColor(Color.BLUE);
        float[] zEnd = projectToIsometric(new Vector3(0, 0, 0.5f));
        canvas.drawLine(centerX, centerY, centerX + zEnd[0], centerY + zEnd[1], axisPaint);
        canvas.drawText("Z", centerX + zEnd[0], centerY + zEnd[1] - 10, textPaint);
    }
}
