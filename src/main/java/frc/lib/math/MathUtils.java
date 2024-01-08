package frc.lib.math;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MathUtils {
    public static boolean equalsWithinError(double targetValue, double currentValue, double error) {
        return Math.abs(currentValue - targetValue) <= error;
    }

    public static double ensureRange(double value, double minValue, double maxValue) {
        if (minValue > value) {
            return minValue;
        } else if (maxValue < value) {
            return maxValue;
        } else {
            return value;
        }
    }

    public static boolean isInRange(double value, double minValue, double maxValue) {
        if (minValue > value) {
            return false;
        } else if (maxValue < value) {
            return false;
        } else {
            return true;
        }
    }


    public static <T> List<List<T>> cartesianProduct(T[][] a) {
        @SuppressWarnings("unchecked")
        List<T>[] b = (List<T>[]) new ArrayList[a.length];
        for (int i = 0; i < b.length; i++) {
            b[i] = Arrays.<T>asList(a[i]);
        }

        return MathUtils.<T>cartesianProduct(0, b);
    }

    private static <T> List<List<T>> cartesianProduct(int i, List<T>[] a) {
        if (i == a.length) {
            List<List<T>> result = new ArrayList<>();
            result.add(new ArrayList<>());
            return result;
        }
        List<List<T>> next = cartesianProduct(i + 1, a);
        List<List<T>> result = new ArrayList<>();
        for (int j = 0; j < a[i].size(); j++) {
            for (int k = 0; k < next.size(); k++) {
                List<T> concat = new ArrayList<>();
                concat.add(a[i].get(j));
                concat.addAll(next.get(k));
                result.add(concat);
            }
        }
        return result;
    }
}
