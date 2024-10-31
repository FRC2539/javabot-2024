package frc.lib.math;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    public static class AnyContainer<T> {
        public T thing;

        public AnyContainer(T thing) {
            this.thing = thing;
        }
    }

    public static class Pipe<T> implements Supplier<T> {
        private Supplier<T> thing;

        public Pipe(Supplier<T> thing) {
            this.thing = thing;
        }

        @Override
        public T get() {
            return thing.get();
        }

        public void set(Supplier<T> thing) {
            this.thing = thing;
        }

        public void set(T thing) {
            this.thing = () -> thing;
        }
    }

    public static class TriggerPipe extends Trigger {
        private Pipe<Boolean> thing;

        public static TriggerPipe of(boolean value) {
            return new TriggerPipe(new Pipe<>(() -> value));
        }

        public static TriggerPipe of(BooleanSupplier valueSupplier) {
            return new TriggerPipe(new Pipe<>(valueSupplier::getAsBoolean));
        }

        private TriggerPipe(Pipe<Boolean> thingPipe) {
            super(thingPipe::get);
            this.thing = thingPipe;
        }

        public void set(boolean value) {
            thing.set(() -> value);
        }

        public void set(BooleanSupplier valueSupplier) {
            thing.set(valueSupplier::getAsBoolean);
        }

        public void setTrue() {
            thing.set(true);
        }

        public void setFalse() {
            thing.set(false);
        }
    }

    public static double accomidateOverflow(double oldRotations, double newRotations, double period) {
        // weird ((x % b) + b) % b to get modulo instead of remainder
        double result = (((newRotations - oldRotations) % period) + period) % period;
        return ((result > period / 2) ? result - period : result) + oldRotations;
    }
}
