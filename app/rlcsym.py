import sympy as sp

# --- symbols ---
t = sp.symbols('t', real=True)
L, C, R = sp.symbols('L C R', positive=True, real=True)
V0 = sp.symbols('V0', real=True)

# natural frequency
w0 = sp.symbols('w0', positive=True, real=True)

# --- functions ---
omega = sp.Function('omega')(t)
a = sp.Function('a')(t)
phi = sp.Function('phi', real=True)(t)

# shifted phase
theta = phi - w0*t

# instantaneous frequency
w = sp.diff(phi, t)

# --- rotating-frame current ---
i = a * sp.exp(-sp.I * theta)

# derivatives
di = sp.diff(i, t)
d2i = sp.diff(i, t, 2)

# --- forcing with constant amplitude ---
V = V0 * sp.exp(-sp.I * theta)
dV = sp.diff(V, t)

# --- RLC equation ---
eq = d2i + (R/L)*di + (1/(L*C))*i - (1/L)*dV

# --- divide out carrier ---
eq_rf = sp.simplify(eq / sp.exp(-sp.I * theta))
eq_rf = sp.expand(eq_rf)

print("\n=== Rotating frame at ω0, constant V0 ===\n")
sp.pprint(eq_rf)

# --- collect terms ---
a_dot = sp.diff(a, t)
a_ddot = sp.diff(a, t, 2)

eq_collected = sp.collect(eq_rf, [a_ddot, a_dot, a])

eq_collected = eq_collected.subs(1/(L*C), w0**2)

print("\n=== Collected form ===\n")
sp.pprint(eq_collected)