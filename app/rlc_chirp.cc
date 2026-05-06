#include <octave/oct.h>
#include <complex>
#include <vector>
#include <cmath>

/* ===================== CORE ===================== */

static inline std::complex<double>
f_func(std::complex<double> z,
       std::complex<double> dz,
       double w,
       double dw,
       double w0,
       double L,
       double R,
       double C)
{
    const std::complex<double> I(0.0, 1.0);

    // i(t) = z(t)*exp(-i*(phase(t)-w_0*t))
    // return ((w*w + I*dw + I*R*w/L - 2.0*w*w0 - I*R*w0/L) * z -
    //        (R/L - 2.0 * I * w + 2.0*I*w0) * dz - I*w/L + I*w0/L);

    // i(t) = z(t)*exp(-i*phase(t))
    return ((w * w + I*dw + I*R*w/L - 1/(C*L)) * z
           - (R/L - 2.0 * I * w) * dz - I * w / L);
    
    // i(t) = z(t)*exp(i*phase(t))
    // return ((w * w - I*dw - I*R*w/L - 1/(C*L)) * z -
    //        (R/L + 2.0 * I * w) * dz - I * w / L);
}

/* ===================== ENTRY ===================== */

DEFUN_DLD(rlc_chirp, args, ,
          "[z, dz] = rlc_chirp(dt, tm, f0, ff, L, R, C, C0)")
{
    if (args.length() != 8)
        error("Usage: [z, dz] = rlc_chirp(dt, tm, f0, ff, L, R, C, C0)");

    /* ---- Parse args ---- */
    double dt = args(0).double_value();
    double tm = args(1).double_value();
    double f0 = args(2).double_value();
    double ff = args(3).double_value();
    double L = args(4).double_value();
    double R = args(5).double_value();
    double C = args(6).double_value();
    double C0 = args(7).double_value();

    if (dt <= 0 || tm <= 0)
        error("dt and tm must be positive");

    /* ---- Derived ---- */
    int N = static_cast<int>(tm / dt) + 1;

    double w0 = 2.0 * M_PI * f0;
    double wf = 2.0 * M_PI * ff;

    double dw = 2.0 * M_PI * (ff - f0) / tm;

    double w_res = 1.0 / std::sqrt(L * C);

    /* ---- Allocate ---- */
    std::vector<std::complex<double>> z(N);
    std::vector<std::complex<double>> dz(N);
    const std::complex<double> I(0.0, 1.0);

    z[0] = I*w0 / (w0*w0 - w_res*w_res - 2.0*I*w0*R/L) / L;
    dz[0] = 0.0;

    /* ---- RK4 loop ---- */
    for (int st = 1; st < N; st++)
    {
        double t = (st - 1) * dt;
        double w = w0 + (wf - w0) * (t / tm);

        std::complex<double> z0 = z[st - 1];
        std::complex<double> dz0 = dz[st - 1];

        auto k1z = dz0;
        auto k1dz = f_func(z0, dz0, w, dw, w0, L, R, C);

        auto k2z = dz0 + 0.5 * dt * k1dz;
        auto k2dz = f_func(z0 + 0.5 * dt * k1z,
                           dz0 + 0.5 * dt * k1dz,
                           w, dw, w0, L, R, C);

        auto k3z = dz0 + 0.5 * dt * k2dz;
        auto k3dz = f_func(z0 + 0.5 * dt * k2z,
                           dz0 + 0.5 * dt * k2dz,
                           w, dw, w0, L, R, C);

        auto k4z = dz0 + dt * k3dz;
        auto k4dz = f_func(z0 + dt * k3z,
                           dz0 + dt * k3dz,
                           w, dw, w0, L, R, C);

        z[st] = z0 + (dt / 6.0) * (k1z + 2.0 * k2z + 2.0 * k3z + k4z);
        dz[st] = dz0 + (dt / 6.0) * (k1dz + 2.0 * k2dz + 2.0 * k3dz + k4dz);
    }

    /* ---- Return ---- */
    ComplexColumnVector z_out(N);
    ComplexColumnVector dz_out(N);

    for (int i = 0; i < N; i++)
    {
        z_out(i) = z[i] - I*C0*(w0+(wf-w0)*(double)i/(double)N);
        dz_out(i) = dz[i];
    }

    octave_value_list out;
    out(0) = z_out;
    out(1) = dz_out;

    return out;
}