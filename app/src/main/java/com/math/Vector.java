package com.math;

public class Vector extends Matrix {
    // Constructor
    public Vector(int size) {
        super(size, 1);
    }

    public Vector(double[][] data) {
        super(data);
        if (getCols() != 1) {
            throw new IllegalArgumentException("Vector must have exactly 1 column");
        }
    }

    public Vector(double[] data) {
        super(data.length, 1);
        for (int i = 0; i < data.length; i++) {
            set(i, 0, data[i]);
        }
    }

    // Copy constructor
    public Vector(Vector other) {
        super(other.getData());
    }

    // Convenience methods
    public double get(int i) {
        return getData()[i][0];
    }

    public void set(int i, double value) {
        set(i, 0, value);
    }

    public int size() {
        return getRows();
    }

    // Override operations to return Vector when possible
    @Override
    public Vector add(Matrix other) {
        return new Vector(super.add(other).getData());
    }

    @Override
    public Vector subtract(Matrix other) {
        return new Vector(super.subtract(other).getData());
    }

    public static Vector scalarMulti(double scale,Vector org) {
        return new Vector(Matrix.scalarMulti(scale,org).getData());
    }

    // Static factory methods
    public static Vector zeros(int size) {
        return new Vector(Matrix.zeros(size, 1).getData());
    }

    public static Vector ones(int size) {
        return new Vector(Matrix.ones(size, 1).getData());
    }
}
