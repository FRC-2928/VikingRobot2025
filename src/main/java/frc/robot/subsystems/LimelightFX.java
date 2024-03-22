package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;

public final class LimelightFX extends SubsystemBase {
	/** The maximum number of modules the FX supports. */
	public static final int maxModules = 32;
	/** The number of layers the FX supports per module. */
	public static final int maxLayers = 2;
	/** The maximum number of total LEDs the FX supports. */
	public static final int maxLEDs = 1024;
	/** The longest a file path can be. Shorter paths conserves memory. */
	public static final int maxFilepath = 32;

	/** An RGBA color. Components are 0-255. */
	public static final class Color {
		public static final Color BLACK = new Color(0);
		public static final Color WHITE = new Color(255);
		public static final Color RED = new Color(255, 0, 0, 255);
		public static final Color GREEN = new Color(0, 255, 0, 255);
		public static final Color BLUE = new Color(0, 0, 255, 255);

		public final int a;
		public final int r;
		public final int g;
		public final int b;

		public Color(final int r, final int g, final int b, final int a) {
			this.a = MathUtil.clamp(a, 0, 255);
			this.r = MathUtil.clamp(r, 0, 255);
			this.g = MathUtil.clamp(g, 0, 255);
			this.b = MathUtil.clamp(b, 0, 255);
		}

		public Color(final int r, final int g, final int b) { this(r, g, b, 255); }

		public Color(final int value) { this(value, value, value); }

		@Override
		public final boolean equals(final Object other) {
			if(this != other) return false;
			if(this instanceof Color) {
				final Color col = (Color) other;
				return this.r == col.r && this.g == col.g && this.b == col.b;
			} else if(other instanceof final int[] arr) {
				return this.r == arr[0] && this.g == arr[1] && this.b == arr[2];
			}
			return false;
		}

		@Override
		public final String toString() { return String.format("0x%02x%02x%02x%02x", this.a, this.r, this.g, this.b); }
	}

	/** An audio tone waveform. */
	public static enum WaveForm {
		Sine(0), Square(1), Sawtooth(2);

		private WaveForm(final int id) { this.id = id; }

		public final int id;
	}

	/** Built-in sounds that can be played directly. */
	public static enum SystemSound {
		Beep0(0),
		Beep1(1),
		Beep2(2),
		Beep3(3),
		Beep4(4),
		Buzz0(5),
		Buzz1(6),
		Buzz2(7),
		Buzz3(8),
		Buzz4(9),
		Explosion(10),
		MenuMove(11),
		MonsterAppear(12),
		MonsterHurt(13),
		Coin(14),
		Swoosh(15),
		Damage(16),
		DogBark(17),
		CarHorn0(18),
		CarHorn1(19);

		private SystemSound(final int id) { this.id = id; }

		public final int id;
	}

	/** An LED module. */
	public static final class Module {
		/** Defines the shape of the module. */
		public static enum Geometry {
			Strip16x1(64, 16, 1), Grid24x12(65, 24, 12), Ring(66, /* unknown */ 0, /* unknown */ 0);

			private Geometry(final int id, final int width, final int height) {
				this.id = id;
				this.width = width;
				this.height = height;
			}

			public final int id;
			public final int width;
			public final int height;
		}

		/** Defines the rotation of the module. */
		public static enum Rotation {
			R0(0), R90(1), R180(2), R270(3);

			private Rotation(final int id) { this.id = id; }

			public final int id;
		}

		Module(final int id, final LimelightFX fx, final Geometry geo, final Rotation rot) {
			this.id = id;
			this.fx = fx;

			this.geo = geo;
			this.rot = rot;
		}

		public final int id;
		public final LimelightFX fx;

		public final Geometry geo;
		public final Rotation rot;

		final Layer[] layers = new Layer[LimelightFX.maxLayers];

		/**
		 * Creates a behavior. Can only be used before {@code LimelightFX.initialize}
		 *
		 * @param kind
		 *                  The behavior class. (Example: {@code LimelightFX.Behavior.BlinkBehavior.class})
		 * @param layer
		 *                  The layer index. Only one behavior can be active per module per layer.
		 * @return The behavior
		 */
		public final <T extends Behavior<T>> T behavior(final Class<T> kind, final int layer) {
			T beh;
			try {
				beh = kind.getConstructor(LimelightFX.class, Module.class, int.class).newInstance(this.fx, this, layer);
			} catch(final Exception e) {
				throw new Error("Behavior creation failed", e);
			}

			this.layers[layer].behaviors.add(beh);

			return beh;
		}
	}

	/** Stores layer data. */
	private static final class Layer {
		Layer(final int id) { this.id = id; }

		/** Numerical index. */
		public final int id;

		/** Behaviors on this layer. */
		final HashSet<Behavior<?>> behaviors = new HashSet<>();
		/** Whether or not this layer had an active behavior last update. */
		boolean wasActive = false;

		/** Helper method to deactivate other behaviors on this layer except for the desired one. */
		final void solo(final Behavior<?> solo) {
			for(final Behavior<?> beh : this.behaviors)
				if(beh != solo) beh.active.set(false);
		}
	}

	/** A behavior on a module and layer */
	public static abstract class Behavior<T extends Behavior<T>> {
		/** A parameter that can be updated to the FX */
		public static final class Param<T> {
			public Param(final T initial) {
				this.value = initial;
				this.dirty = true;
			}

			/** The internal value */
			private T value;
			/** The dirty flag, if true, the parameter will be sent next update */
			public boolean dirty;

			/** Get the value of the parameter */
			public final T get() { return this.value; }

			/** Get the value of the parameter and mark it dirty */
			public final T mut() {
				this.dirty = true;
				return this.value;
			}

			/** Set the value of the parameter and mark it dirty if the value is not equal */
			public final boolean set(final T value) {
				if(this.value.equals(value)) return false;
				this.value = value;
				this.dirty = true;
				return true;
			}

			@Override
			public final String toString() { return this.value.toString(); }
		}

		/** A simple configurable blink pattern. */
		public static class BlinkBehavior extends Behavior<BlinkBehavior> {
			BlinkBehavior(final LimelightFX fx, final Module module, final int layer) {
				super(fx, module, layer);

				this.id = 0;
				this.params = new Param<?>[] {
					this.colorA,
					this.colorB,

					this.timeOnA,
					this.timeOffA,
					this.timeBetween,
					this.timeOnB,
					this.timeOffB,
					this.timeRepeat,

					this.blinkCountA,
					this.blinkCountB,
					this.repeatCount,

					this.fadeInA,
					this.fadeOutA,
					this.fadeInB,
					this.fadeOutB, };
			}

			/** The A color. */
			public Param<Color> colorA = new Param<>(new Color(255, 127, 255));
			/** The B color. */
			public Param<Color> colorB = new Param<>(new Color(127, 0, 255));

			/** The time color A should be on for. Time is in 100ths of a second. */
			public Param<Integer> timeOnA = new Param<>(100);
			/** The time color A should be off for. Time is in 100ths of a second. */
			public Param<Integer> timeOffA = new Param<>(100);
			/** The time between color A and color B. Time is in 100ths of a second. */
			public Param<Integer> timeBetween = new Param<>(100);
			/** The time color A should be on for. Time is in 100ths of a second. */
			public Param<Integer> timeOnB = new Param<>(30);
			/** The time color A should be off for. Time is in 100ths of a second. */
			public Param<Integer> timeOffB = new Param<>(30);
			/** The time between color B and color A (if set to repeat). Time is in 100ths of a second. */
			public Param<Integer> timeRepeat = new Param<>(100);

			/** How many times color A should blink. */
			public Param<Integer> blinkCountA = new Param<>(1);
			/** How many times color B should blink. */
			public Param<Integer> blinkCountB = new Param<>(1);
			/** How many times the entire sequence should repeat. Set to 0 to repeat infinitely. */
			public Param<Integer> repeatCount = new Param<>(0);

			/** Time to fade in color A */
			public Param<Integer> fadeInA = new Param<>(50);
			/** Time to fade out color A */
			public Param<Integer> fadeOutA = new Param<>(50);
			/** Time to fade in color B */
			public Param<Integer> fadeInB = new Param<>(1);
			/** Time to fade out color B */
			public Param<Integer> fadeOutB = new Param<>(30);

			@Override
			protected final void cloneFrom(final BlinkBehavior fused) {
				this.colorA = fused.colorA;
				this.colorB = fused.colorB;

				this.timeOnA = fused.timeOnA;
				this.timeOffA = fused.timeOffA;
				this.timeBetween = fused.timeBetween;
				this.timeOnB = fused.timeOnB;
				this.timeOffB = fused.timeOffB;
				this.timeRepeat = fused.timeRepeat;

				this.blinkCountA = fused.blinkCountA;
				this.blinkCountB = fused.blinkCountB;
				this.repeatCount = fused.repeatCount;

				this.fadeInA = fused.fadeInA;
				this.fadeOutA = fused.fadeOutA;
				this.fadeInB = fused.fadeInB;
				this.fadeOutB = fused.fadeOutB;
			}
		}

		// public static class ChevronsBehavior extends Behavior {
		// 	public enum Direction {
		// 		Up(0), Down(2), Left(1), Right(3);

		// 		public final int value;

		// 		private Direction(final int value) { this.value = value; }
		// 	}

		// 	/** The Color for the A chevrons */
		// 	public Color colorA = new Color(255, 127, 255);
		// 	/** The Color for the B chevrons */
		// 	public Color colorB = new Color(127, 0, 255);
		// 	/** The thickness for the A chevrons in pixels */
		// 	public int thicknessA = 4;
		// 	/** The thickness for the B chevrons in pixels */
		// 	public int thicknessB = 2;
		// 	/**
		// 	 * The speed the chevrons should move at
		// 	 *
		// 	 * Measured in pixels per tick (100 ticks/sec)
		// 	 */
		// 	public int speed = 1;
		// 	/** The direction the chevrons should move in */
		// 	public Direction dir = Direction.Up;

		// 	@Override
		// 	public int id() { return 5; }

		// 	@Override
		// 	public String[] params() {
		// 		return new String[] {
		// 			this.colorA.toString(),
		// 			this.colorB.toString(),
		// 			Integer.toString(this.thicknessA),
		// 			Integer.toString(this.thicknessB),
		// 			Integer.toString(this.speed * 255),
		// 			Integer.toString(this.dir.value) };
		// 	}
		// }

		// public static class CountdownBehavior extends Behavior {
		// 	/** The Color for the countdown */
		// 	//public Color colorA = new Color(255);
		// 	public Color colorA = new Color(255);
		// 	/** The total countdown time in milliseconds */
		// 	public int ms = 15000;
		// 	/** Whether the countdown should stop when it hits zero */
		// 	public boolean stopAtZero = true;
		// 	/** Whether the countdown should be paused */
		// 	public boolean paused = false;

		// 	@Override
		// 	public int id() { return 11; }

		// 	@Override
		// 	public String[] params() {
		// 		return new String[] {
		// 			this.colorA.toString(),
		// 			Integer.toString(this.ms),
		// 			this.stopAtZero ? "1" : "0",
		// 			this.paused ? "1" : "0" };
		// 	}
		// }

		protected Behavior(final LimelightFX fx, final Module module, final int layer) {
			this.fx = fx;
			this.module = module;
			this.layer = module.layers[layer];

			this.fused.add(this);
		}

		public final LimelightFX fx;
		public final Module module;
		public final Layer layer;

		int id;
		Param<?>[] params;
		Param<Boolean> active = new Param<>(false);
		HashSet<Behavior<?>> fused = new HashSet<>();

		@SuppressWarnings("unchecked")
		public final Behavior<T> on(final Module module, final int layer) {
			if(this.fx.initialized)
				throw new Error("LimelightFX: Cannot create/modify Behaviors after initialization is complete");

			T fused;
			try {
				final Constructor<T> constructor = ((Class<T>) this.getClass())
					.getConstructor(LimelightFX.class, Module.class, int.class);
				constructor.setAccessible(true);
				fused = constructor.newInstance(this.fx, module, layer);
			} catch(final Exception e) {
				throw new Error(e);
			}

			this.module.layers[layer].behaviors.add(fused);

			this.fuse(fused);
			fused.cloneFrom(fused);
			fused.params = this.params;

			return this;
		}

		public final void fuse(final Behavior<?> with) {
			if(this.fx.initialized)
				throw new Error("LimelightFX: Cannot create/modify Behaviors after initialization is complete");

			this.fused.addAll(with.fused);
			with.fused = this.fused;
			with.active = this.active;

			final HashSet<Layer> seen = new HashSet<>();

			for(final Behavior<?> fused : this.fused) {
				if(fused.fx != this.fx)
					throw new Error("LimelightFX: Cannot fuse with a Behavior that is on a different FX core!");
				if(seen.contains(fused.layer))
					throw new Error(
						"LimelightFX: Multiple fused Behaviors on the same Module and Layer! This state is invalid, as none of them may activate!"
					);
				seen.add(fused.layer);
			}
		}

		public final void activate() {
			if(!this.active.set(true)) return;

			for(final Behavior<?> fused : this.fused)
				fused.layer.solo(fused);
		}

		public final void deactivate() { this.active.set(false); }

		/**
		 * Creates a command that starts and ends this behavior
		 */
		public final Command command() {
			return new FunctionalCommand(this::activate, () -> {
			}, interrupted -> this.deactivate(), () -> false);
		}

		final String paramstr() {
			String current = "";

			for(final Param<?> param : this.params)
				current += " " + param.toString();

			return current;
		}

		protected abstract void cloneFrom(T fused);
	}

	private static final class Selector {
		Selector(final Supplier<Behavior<?>> selector) { this.selector = selector; }

		public final Supplier<Behavior<?>> selector;

		private Behavior<?> last = null;

		final void update() {
			final Behavior<?> beh = this.selector.get();

			if(beh != null) beh.activate();
			else if(this.last != null) this.last.deactivate();

			this.last = beh;
		}
	}

	// IMPLEMENTATION //

	private boolean initialized;

	private SerialPort serial;

	private final ArrayList<Module> modules = new ArrayList<>();
	private final ArrayList<Selector> selectors = new ArrayList<>();

	public final Module module(final Module.Geometry geo, final Module.Rotation rot) {
		if(this.initialized) throw new Error("LimelightFX: Cannot create Modules after initialization is complete");

		final Module module = new Module(this.modules.size(), this, geo, rot);
		for(int i = 0; i < LimelightFX.maxLayers; i++)
			module.layers[i] = new Layer(i);

		this.modules.add(module);

		return module;
	}

	public final void selector(final Supplier<Behavior<?>> selector) {
		if(this.initialized) throw new Error("LimelightFX: Cannot create Selectors after initialization is complete");

		this.selectors.add(new Selector(selector));
	}

	/**
	 * Completes the initialization process
	 *
	 * @param port
	 *                 The serial port that is connected to the LimelightFX Core
	 */
	public final void initialize(final SerialPort.Port port) {
		if(this.initialized) throw new Error("LimelightFX: Cannot initialize, already initialized!");
		this.initialized = true;

		System.err.println("LimelightFX: Initializing...");
		this.serial = new SerialPort(115200, port);
		this.serial.reset();
		this.serial.setTimeout(0.5);
		this.serial.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
		System.err.println("LimelightFX: Transmitting Modules...");
		this.write("init 1");
		for(final Module module : this.modules)
			this.write("module ", module.geo.id, module.geo.width, module.geo.height, module.rot.id);
		this.write("start");
		System.err.println("LimelightFX: Initialized");
	}

	/**
	 * Writes data to the port and disables the FX if write fails
	 */
	private final void write(final String data) {
		if(!this.initialized) throw new Error("LimelightFX: Cannot write before initialization");

		try {
			if(this.serial == null) return;

			final int n = this.serial.writeString(data + "\n");

			if(n == 0) this.disable("cannot write");
		} catch(final Exception e) {
			this.disable(e.getMessage());
		}
	}

	/**
	 * Writes data to the port and disables the FX if write fails
	 */
	private final void write(final String data, final Object... args) {
		try {
			if(this.serial == null) return;

			final int n = this.serial.writeString(String.format(data, args) + "\n");

			if(n == 0) this.disable("cannot write");
		} catch(final Exception e) {
			this.disable(e.getMessage());
		}
	}

	/**
	 * Disable the FX
	 */
	public final void disable(final String reason) {
		if(this.serial == null) return;

		this.serial.close();
		this.serial = null;
		System.err.println("LimelightFX: FX disabled (" + reason + ")");
	}

	public final
		void
		sound(final WaveForm sound, final int freq, final int duration, final int channel, final double volume) {
		this
			.write(
				"tone %d %d %d %d %d",
				freq,
				duration,
				sound.id,
				channel,
				MathUtil.clamp(Math.round(volume * 255), 0, 255)
			);
	}

	public final void sound(final SystemSound sound, final int channel, final int loops, final double volume) {
		this.write("sound %d %d %d %d", sound.id, channel, loops, MathUtil.clamp(Math.round(volume * 255), 0, 255));
	}

	public final void sound(final String filename, final int channel, final double volume) {
		this.write("stream %s %d %d", filename, channel, MathUtil.clamp(Math.round(volume * 255), 0, 255));
	}

	@Override
	public final void periodic() {
		if(!this.initialized || this.serial == null) return; // exploded

		for(final Selector selector : this.selectors)
			selector.update();

		for(final Module mod : this.modules)
			for(final Layer layer : mod.layers) {
				boolean has_active = false;

				for(final Behavior<?> beh : layer.behaviors) {
					has_active |= beh.active.value;

					if(!beh.active.value) continue;

					if(beh.active.dirty) this.write("beh %d %d %d%s", beh.module, beh.layer, beh.id, beh.paramstr());
					else for(final Behavior.Param<?> param : beh.params)
						if(param.dirty) this.write("p %d %d %d %s", beh.module, beh.layer, param);
				}

				if(!has_active && layer.wasActive) this.write("beh %d %d -1", mod.id, layer.id);
			}

		for(final Module mod : this.modules)
			for(final Layer layer : mod.layers)
				for(final Behavior<?> beh : layer.behaviors)
					for(final Behavior.Param<?> param : beh.params)
						param.dirty = false;
	}
}
