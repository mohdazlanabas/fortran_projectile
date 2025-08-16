// projectile_final_frame_autov0_minimum.cpp
// Auto-calculate the MINIMUM projectile launch speed needed to intercept a target
// moving East at a user-entered speed. No artificial multipliers are applied.
// Inputs: target X in [-10,10] mi, target Y in [1,10] mi, target speed in [50,500] mph.
// Output order: Kinematic Solution -> Illustrative Launch Effort -> Physics formulas -> Final Frame (full duration, auto-scaled)

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

struct State { double x, y; };

static constexpr double G_MI = 32.174 / 5280.0; // miles/s^2 ≈ 0.006091
static constexpr double PI   = 3.14159265358979323846;

// Visual-only vertical exaggeration (does not affect physics)
static constexpr double Y_STRETCH = 3.0;

// “Launch effort” (illustrative only)
static constexpr double MASS_KG     = 1.0;
static constexpr double ACCEL_LEN_M = 50.0;
static constexpr double MILES_TO_M  = 1609.344;

// Canvas size
static constexpr int CANVAS_W = 60;
static constexpr int CANVAS_H = 18;

// --- Motion models ---
static inline State pos_proj(double v0_mis, double angle_deg, double t) {
    double a  = angle_deg * PI / 180.0;
    double vx = v0_mis * std::cos(a);
    double vy = v0_mis * std::sin(a);
    return { vx * t, vy * t - 0.5 * G_MI * t * t };
}
static inline State pos_target(double tx0, double ty0, double vT_mis, double t) {
    return { tx0 + vT_mis * t, ty0 }; // heading East; y fixed
}

// Components at time T (launcher at origin):
// Cx = tx0/T + vT,  Cy = ty0/T + 0.5*g*T  ;  required speed = sqrt(Cx^2 + Cy^2)
struct Comp { double Cx, Cy; };
static inline Comp components(double tx0, double ty0, double vT_mis, double T) {
    return { tx0 / T + vT_mis, ty0 / T + 0.5 * G_MI * T };
}

// Golden-section minimization (for 1-D)
template <typename Fun>
static double golden_minimize(Fun G, double a, double b, int iters = 140) {
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    const double invphi = 1.0 / phi;
    double c = b - (b - a) * invphi;
    double d = a + (b - a) * invphi;
    double fc = G(c), fd = G(d);
    for (int i = 0; i < iters; ++i) {
        if (fc < fd) { b = d; d = c; fd = fc; c = b - (b - a) * invphi; fc = G(c); }
        else         { a = c; c = d; fc = fd; d = a + (b - a) * invphi; fd = G(d); }
    }
    return (fc < fd) ? c : d;
}

int main() {
    using std::cin; using std::cout;

    cout << "=== 2D Projectile vs. Moving Target (auto minimum launch speed) ===\n";
    cout << "Distances: miles | Time: s | Speeds in mph and mi/s | g = "
         << std::fixed << std::setprecision(6) << G_MI << " mi/s^2\n\n";

    // ---- Inputs with validation ----
    double tx0, ty0, vT_mph_user;

    do {
        cout << "Enter target starting X position (miles) [-10 to 10]: ";
        if (!(cin >> tx0)) return 0;
        if (tx0 < -10.0 || tx0 > 10.0) cout << "❌ Invalid. X must be between -10 and 10 miles.\n";
    } while (tx0 < -10.0 || tx0 > 10.0);

    do {
        cout << "Enter target starting Y position (miles) [1 to 10]: ";
        if (!(cin >> ty0)) return 0;
        if (ty0 < 1.0 || ty0 > 10.0) cout << "❌ Invalid. Y must be between 1 and 10 miles.\n";
    } while (ty0 < 1.0 || ty0 > 10.0);

    do {
        cout << "Enter target speed (mph) [50 to 500]: ";
        if (!(cin >> vT_mph_user)) return 0;
        if (vT_mph_user < 50.0 || vT_mph_user > 500.0)
            cout << "❌ Invalid. Speed must be between 50 and 500 mph.\n";
    } while (vT_mph_user < 50.0 || vT_mph_user > 500.0);

    const double vT_mis_user = vT_mph_user / 3600.0;

    // ---- Auto-calculate MINIMUM launch speed and intercept time ----
    auto Greq = [&](double T)->double {
        Comp c = components(tx0, ty0, vT_mis_user, T);
        return std::sqrt(c.Cx*c.Cx + c.Cy*c.Cy);
    };

    const double tmin = 1e-6;
    double tmax = 1.0;
    for (int i=0;i<25;++i) tmax *= 2.0;

    double T_star = golden_minimize(Greq, tmin, tmax);
    Comp comp_star = components(tx0, ty0, vT_mis_user, T_star);
    double v0_mis  = std::sqrt(comp_star.Cx*comp_star.Cx + comp_star.Cy*comp_star.Cy); // minimal speed in mi/s
    double v0_mph  = v0_mis * 3600.0;                                                   // mph
    double ang_deg = std::atan2(comp_star.Cy, comp_star.Cx) * 180.0 / PI;              // launch angle

    State Tf_user = pos_target(tx0, ty0, vT_mis_user, T_star);

    // ---- Kinematic Solution ----
    cout << "\n--- Kinematic Solution (exact intercept) ---\n";
    cout << std::fixed << std::setprecision(3);
    cout << "Target initial position:   (" << tx0 << ", " << ty0 << ") mi\n";
    cout << "Target speed (entered):    " << vT_mph_user << " mph (" << vT_mis_user << " mi/s)\n";
    cout << "Projectile launch speed:   " << v0_mph << " mph (" << v0_mis << " mi/s)\n";
    cout << "Launch angle:              " << ang_deg << " deg\n";
    cout << "Time to meet (solved):     " << T_star << " s\n";
    cout << "Target final position:     (" << Tf_user.x << ", " << Tf_user.y << ") mi\n";

    // ---- Launch effort ----
    const double v0_mps = v0_mis * MILES_TO_M;
    const double a_mps2 = (ACCEL_LEN_M > 0.0) ? (v0_mps*v0_mps)/(2.0*ACCEL_LEN_M) : 0.0;
    const double F_N    = MASS_KG * a_mps2;
    const double J_Ns   = MASS_KG * v0_mps;

    cout << "\n--- Illustrative Launch Effort ---\n";
    cout << "Assumed mass:              " << MASS_KG << " kg\n";
    cout << "Accel distance:            " << ACCEL_LEN_M << " m\n";
    cout << "Avg acceleration:          " << a_mps2 << " m/s^2\n";
    cout << "Avg force:                 " << F_N << " N\n";
    cout << "Impulse (m*v):             " << J_Ns << " N·s\n";

    // ---- Physics formulas ----
    cout << "\n--- Physics formulas used ---\n";
    cout << "Projectile (no drag):      x(t)=v0 cosθ t,  y(t)=v0 sinθ t - ½ g t²\n";
    cout << "Target (uniform East):     x(t)=x0 + v_T t, y(t)=y0\n";
    cout << "Interception (auto v0):    minimize G(T)=√[(x0/T+v_T)² + (y0/T+½gT)²]\n";
    cout << "                           v0=G(T*),  θ=atan2(y0/T*+½gT*, x0/T*+v_T)\n";

    // ---- Final Frame ----
    const double T_plot = std::max(T_star, 0.5);

    const int N = 1500;
    std::vector<State> proj, targ;
    proj.reserve(N); targ.reserve(N);
    for (int i = 0; i < N; ++i) {
        double t = (double)i * T_plot / std::max(1, N - 1);
        proj.push_back(pos_proj(v0_mis, ang_deg, t));
        targ.push_back(pos_target(tx0, ty0, vT_mis_user, t));
    }

    auto toPlot = [](const State& s){ return State{ s.x, s.y * Y_STRETCH }; };
    double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;
    auto upd = [&](const State& sP){
        State s = toPlot(sP);
        xmin = std::min(xmin, s.x); xmax = std::max(xmax, s.x);
        ymin = std::min(ymin, s.y); ymax = std::max(ymax, s.y);
    };
    for (const auto& s : proj) upd(s);
    for (const auto& s : targ) upd(s);
    double mx = (xmax - xmin) * 0.1 + 0.5;
    double my = (ymax - ymin) * 0.1 + 0.5;
    xmin -= mx; xmax += mx;
    ymin = std::min(0.0, ymin - my);
    ymax += my;

    const int W = CANVAS_W, H = CANVAS_H;
    auto toScreen = [&](const State& sWorld){
        State s = toPlot(sWorld);
        int col = (int)std::llround((s.x - xmin) / std::max(1e-12, (xmax - xmin)) * (W - 1));
        int row = (int)std::llround((1.0 - (s.y - ymin) / std::max(1e-12, (ymax - ymin))) * (H - 1));
        col = std::max(0, std::min(W - 1, col));
        row = std::max(0, std::min(H - 1, row));
        return std::pair<int,int>(row, col);
    };

    std::vector<std::string> grid(H, std::string(W, ' '));
    std::vector<std::vector<int>> prio(H, std::vector<int>(W, -1));
    auto put = [&](int r, int c, char ch, int p){
        if (r<0||r>=H||c<0||c>=W) return;
        if (p >= prio[r][c]) { grid[r][c] = ch; prio[r][c] = p; }
    };
    enum {P_GROUND=0, P_PPATH=1, P_TPATH=2, P_POEND=3, P_TEND=4};

    for (int cc = 0; cc < W; ++cc) {
        double x = xmin + (xmax - xmin) * (double)cc / (W - 1);
        auto s0 = toScreen({x, 0.0});
        put(s0.first, cc, '_', P_GROUND);
    }
    for (const auto& s : targ) {
        auto p = toScreen(s);
        put(p.first, p.second, '-', P_TPATH);
    }
    std::vector<bool> row_used(H, false);
    for (const auto& s : proj) {
        auto p = toScreen(s);
        if (!row_used[p.first]) { put(p.first, p.second, '.', P_PPATH); row_used[p.first] = true; }
    }
    auto Tstart = toScreen(targ.front());
    auto Tend   = toScreen(targ.back());
    put(Tstart.first, Tstart.second, 'X', P_TEND);
    put(Tend.first,   Tend.second,   'X', P_TEND);

    auto Pstart = toScreen(proj.front());
    auto Pend   = toScreen(proj.back());
    put(Pstart.first, Pstart.second, 'O', P_POEND);
    put(Pend.first,   Pend.second,   'O', P_POEND);

    cout << "\n--- Final Frame (full duration, auto-scaled) ---\n";
    cout << "Plotted duration T = " << std::setprecision(3) << T_plot << " s\n";
    cout << "Legend: _ ground, - target path, X target start/end, . projectile path, O projectile start/end\n\n";
    for (auto& line : grid) cout << line << "\n";

    cout << "\n";
    return 0;
}

