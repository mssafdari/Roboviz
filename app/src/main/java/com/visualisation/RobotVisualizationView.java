package com.visualisation;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;
import android.view.MotionEvent;

import java.util.ArrayList;
import java.util.List;
import com.roboviz.Joint;
import com.roboviz.Robot;
import com.kinematics.forwardKinematics;
import com.math.Vector;
import com.math.Vector3;

public class RobotVisualizationView extends View {
    private Robot robot;
    private float scale = 80f; // pixels per meter
    private int selectedJoint = -1;
    private float originX,originY;
    private int numJoints;
    public Vector thetaList;
    
    private float endEffectorX=f(robot.joints.get(robot.joints.size()-1).location.x);
    private float endEffectorY=f(robot.joints.get(robot.joints.size()-1).location.y);
    
    
    private Paint jointPaint, linkPaint, textPaint, gridPaint;

    public RobotVisualizationView(Context context,Robot robo) {
        super(context);
        robot=robo;
        numJoints=robo.joints.size();
        init();
    }

    public RobotVisualizationView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        jointPaint = new Paint();
        jointPaint.setColor(Color.BLUE);
        jointPaint.setStyle(Paint.Style.FILL);

        linkPaint = new Paint();
        linkPaint.setColor(Color.BLACK);
        linkPaint.setStrokeWidth(6);
        linkPaint.setStyle(Paint.Style.STROKE);

        textPaint = new Paint();
        textPaint.setColor(Color.RED);
        textPaint.setTextSize(20);

        gridPaint = new Paint();
        gridPaint.setColor(Color.LTGRAY);
        gridPaint.setStrokeWidth(1);
       
    }


    public void setupRobotFromURDF(List<Joint> joints, List<Double> lengths) {
        // This method would parse your URDF and setup the robot properly
        // For now, we'll keep the default configuration
    }

    public void setJointAngles(List<Double> angles) {
        
        if (angles.size() >= numJoints) {
            for (int i = 0; i < numJoints; i++) {
                robot.joints.get(i).setCurrentPosition(angles.get(i));
            }
            calculateForwardKinematics();
            invalidate();
        }
    }

    public void setJointAngle(int index, double angle) {
        if (index >= 0 && index < robot.joints.size()) {
            robot.joints.get(index).setCurrentPosition(angle);
            calculateForwardKinematics();
            invalidate();
        }
    }

    public void resetToDefault() {
        robot.reset();
        calculateForwardKinematics();
        invalidate();
    }

    private void calculateForwardKinematics() {
        forwardKinematics.FKinSpace(robot.M0i.get(robot.M0i.size()),robot.Slist,thetaList,false);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        // Set origin at center of view
        originX = w / 2f;
        originY = h / 2f;

        // Adjust scale based on view size
        scale = Math.min(w, h) / 12f;
        calculateForwardKinematics();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        // Draw grid
        drawGrid(canvas);

        // Draw robot
        drawRobot(canvas);

        // Draw coordinate system
        drawCoordinateSystem(canvas);

        // Draw end effector info
        drawEndEffectorInfo(canvas);
    }

    private void drawGrid(Canvas canvas) {
        int width = getWidth();
        int height = getHeight();

        // Draw grid lines
        for (int i = 0; i < width; i += 50) {
            canvas.drawLine(i, 0, i, height, gridPaint);
        }
        for (int i = 0; i < height; i += 50) {
            canvas.drawLine(0, i, width, i, gridPaint);
        }

        // Draw axes
        Paint axisPaint = new Paint();
        axisPaint.setColor(Color.BLACK);
        axisPaint.setStrokeWidth(3);

        canvas.drawLine(0, originY, width, originY, axisPaint); // X-axis
        canvas.drawLine(originX, 0, originX, height, axisPaint); // Y-axis

        // Draw origin point
        Paint originPaint = new Paint();
        originPaint.setColor(Color.RED);
        originPaint.setStyle(Paint.Style.FILL);
        canvas.drawCircle(originX, originY, 8, originPaint);
    }
    
    private float f(double d){
        return (float)d;
    }

    private void drawRobot(Canvas canvas) {
        Vector3 startJoint;
        Vector3 endJoint;
        Vector3 root=robot.root.parentJoint.location;

        // Draw base
        Paint basePaint = new Paint();
        basePaint.setColor(Color.DKGRAY);
        basePaint.setStyle(Paint.Style.FILL);
        canvas.drawCircle(f(root.x),f(root.y), 15, basePaint);

        // Iterate through joints and links
        for (int i = 1; i < numJoints; i++) {
            startJoint=robot.joints.get(i).location;
            endJoint=robot.joints.get(i+1).location;

            // Draw link
            canvas.drawLine(f(startJoint.x),f(startJoint.y), f(endJoint.x), f(endJoint.y), linkPaint);

            // Draw joint
            Paint jointPaintLocal = new Paint();
            jointPaintLocal.setColor(i == selectedJoint ? Color.RED : Color.BLUE);
            jointPaintLocal.setStyle(Paint.Style.FILL);
            canvas.drawCircle(f(startJoint.x), f(startJoint.y), 12, jointPaintLocal);

            // Draw joint number/label
            canvas.drawText(String.valueOf(i), f(startJoint.x) - 10, f(startJoint.y) - 20, textPaint);

            // Draw angle arc (for revolute joints)
            /*if (robot.joints.get(i).type!=robot.jointType.PRISMATIC && i < robot.joints.size()) {
                drawAngleArc(canvas, f(startJoint.x), f(startJoint.y), (float)(totalAngle), (float)(angle), 30);
            }*/
        }

        // Draw end effector
        float endEffectorX=f(robot.joints.get(robot.joints.size()-1).location.x);
        float endEffectorY=f(robot.joints.get(robot.joints.size()-1).location.y);
        
        Paint endEffectorPaint = new Paint();
        endEffectorPaint.setColor(Color.GREEN);
        endEffectorPaint.setStyle(Paint.Style.FILL);
        canvas.drawCircle(endEffectorX,endEffectorY, 15, endEffectorPaint);

        // Draw end effector label
        canvas.drawText("End Effector", endEffectorX - 40, endEffectorY - 25, textPaint);

        // Draw tool tip
        /*if (robot.joints.size() > 0) {
            double toolLength = linkLengths.get(jointAngles.size() - 1);
            float toolX = x + (float) (toolLength * Math.cos(angle));
            float toolY = y + (float) (toolLength * Math.sin(angle));

            Paint toolPaint = new Paint();
            toolPaint.setColor(Color.RED);
            toolPaint.setStrokeWidth(4);
            canvas.drawLine(x, y, toolX, toolY, toolPaint);
        }*/
    }

    /*private void drawAngleArc(Canvas canvas, float cx, float cy, float startAngle, float endAngle, float radius) {
        Paint arcPaint = new Paint();
        arcPaint.setColor(Color.GRAY);
        arcPaint.setStyle(Paint.Style.STROKE);
        arcPaint.setStrokeWidth(2);

        float startDeg = (float) Math.toDegrees(startAngle);
        float sweepDeg = (float) Math.toDegrees(endAngle - startAngle);

        // Android canvas uses degrees, 0 is East, clockwise is positive
        // We need to convert our coordinate system
        startDeg = -startDeg + 90; // Adjust for our coordinate system
        sweepDeg = -sweepDeg;

        canvas.drawArc(cx - radius, cy - radius, cx + radius, cy + radius, 
                       startDeg, sweepDeg, false, arcPaint);
    }*/

    private void drawCoordinateSystem(Canvas canvas) {
        float offset = 50;
        float size = 100;

        // Draw x-axis arrow
        canvas.drawLine(offset, originY, offset + size, originY, new Paint() {{
                    setColor(Color.RED);
                    setStrokeWidth(3);
                }});
        canvas.drawText("X", offset + size + 10, originY + 5, textPaint);

        // Draw y-axis arrow
        canvas.drawLine(originX, offset, originX, offset + size, new Paint() {{
                    setColor(Color.GREEN);
                    setStrokeWidth(3);
                }});
        canvas.drawText("Y", originX + 10, offset + size + 5, textPaint);
    }

    private void drawEndEffectorInfo(Canvas canvas) {
        Paint infoPaint = new Paint();
        infoPaint.setColor(Color.BLACK);
        infoPaint.setTextSize(24);

        String info = String.format("End Effector: (%.2f, %.2f)", 
                                    (endEffectorX - originX) / scale, 
                                    -(endEffectorY - originY) / scale);
        canvas.drawText(info, 20, 40, infoPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float x = event.getX();
        float y = event.getY();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                // Check if user touched a joint
                selectedJoint = getTouchedJoint(x, y);
                if (selectedJoint >= 0) {
                    return true;
                }
                break;
            case MotionEvent.ACTION_MOVE:
                if (selectedJoint >= 0 && selectedJoint < robot.joints.size()) {
                    // Calculate angle from joint position
                    float jointX = originX;
                    float jointY = originY;
                    // Find the position of the selected joint
                    // (simplified - you'd need to compute actual joint positions)

                    // For simplicity, we'll use a basic angle calculation
                    double newAngle = Math.atan2(-(y - jointY), x - jointX);
                    robot.joints.get(selectedJoint).setCurrentPosition(newAngle);
                    calculateForwardKinematics();
                    invalidate();
                    return true;
                }
                break;
            case MotionEvent.ACTION_UP:
                selectedJoint = -1;
                break;
        }

        return super.onTouchEvent(event);
    }

    private int getTouchedJoint(float x, float y) {
        // Simplified - would need to calculate joint positions
        // For now, return -1 (no joint selected)
        return -1;
    }

    // Get current joint angles
    public Vector getCurrentJointAngles() {
        Vector jointAngles=new Vector(robot.joints.size());
        for(int i=0;i<robot.joints.size();i++){
            jointAngles.set(i,robot.joints.get(i).getPos());
        }
        return jointAngles;
    }

    // Get end effector position
    public float[] getEndEffectorPosition() {
        return new float[]{endEffectorX, endEffectorY};
    }
}
