// projectile_final_frame_symbols_v14.cpp
// ASCII final-frame; Output order: Kinematic → Launch Effort → Formulas → Final Frame
// Symbols: _ ground, - target path, X target start/end (+ labels), . projectile path (max 1 dot per row), O projectile start/end
// Visual-only vertical exaggeration so curvature is clear.
// NOTE: Target speed entered by user is multiplied by 1000 for the simulation.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

struct State { double x, y; };

static constexpr double G_MI = 32.174 / 5280.0; // miles/s^2
static constexpr double PI   = 3.14159265358979323846;

// Visual-only vertical exaggeration (does not affect physics)
static constexpr double Y_STRETCH = 3.0;

// Illustrative "launch effort" assumptions (purely educational)
static constexpr double MASS_KG     = 1.0;
static constexpr double ACCEL_LEN_M = 50.0;
static constexpr double MILES_TO_M  = 1609.344;

static inline State pos_proj(double v0_mis, double angle_deg, double t) {
    double a = angle_deg * PI / 180.0;
    double vx = v0_mis * std::cos(a);
    double vy = v0_mis * std::sin(a);
    return { vx * t, vy * t - 0.5 * G_MI * t * t };
}
static inline State pos_target(double tx0, double ty0, double vT_mis, double t) {
    return { tx0 + vT_mis * t, ty0 }; // heading East
}

int main() {
    using std::cin; using std::cout;

    cout << "=== 2D Projectile vs. Moving Target (Final Frame, 10 s) ===\n";
    cout << "Distances: miles | Time: s | Speeds in mph and mi/s | g = "
         << std::fixed << std::setprecision(6) << G_MI << " mi/s^2\n\n";

    // === USER INPUT ===
    double tx0, ty0, vT_mph;
    cout << "Enter target starting X position (miles): ";
    if (!(cin >> tx0)) return 0;
    cout << "Enter target starting Y position (miles): ";
    if (!(cin >> ty0)) return 0;
    cout << "Enter target speed (mph): ";
    if (!(cin >> vT_mph)) return 0;

    const double T = 10.0; // fixed simulation time

    // Store user-entered speed, then multiply by 1000 for the simulation
    const double vT_mph_user = vT_mph;
    vT_mph *= 1000.0;                       // << requirement
    const double vT_mis = vT_mph / 3600.0;  // convert to mi/s after scaling

    // Target final position at T
    State Tf = pos_target(tx0, ty0, vT_mis, T);

    // Compute projectile speed/angle to meet target at T (no drag)
    const double Xc = Tf.x / T;
    const double Yc = (Tf.y + 0.5 * G_MI * T * T) / T;
    const double v0_mis   = std::sqrt(Xc*Xc + Yc*Yc);
    const double v0_mph   = v0_mis * 3600.0;
    const double angle_deg = std::atan2(Yc, Xc) * 180.0 / PI;

    // Build trajectories (fine sampling)
    std::vector<State> proj, targ;
    const double dt = 0.01;
    for (double t = 0.0; t <= T + 1e-9; t += dt) {
        proj.push_back(pos_proj(v0_mis, angle_deg, t));
        targ.push_back(pos_target(tx0, ty0, vT_mis, t));
    }

    // 1) Kinematic Solution
    cout << "\n--- Kinematic Solution (no drag) ---\n";
    cout << std::fixed << std::setprecision(3);
    cout << "Target initial position: (" << tx0 << ", " << ty0 << ") mi\n";
    cout << "Target final position:   (" << Tf.x << ", " << Tf.y << ") mi\n";
    cout << "Target speed (entered):  " << vT_mph_user << " mph  (" << vT_mph_user/3600.0 << " mi/s)\n";
    cout << "Target speed (simulated):" << vT_mph      << " mph  (" << vT_mis                << " mi/s)\n";
    cout << "Projectile launch speed: " << v0_mph << " mph  (" << v0_mis << " mi/s)\n";
    cout << "Launch angle:            " << angle_deg << " deg\n";
    cout << "Time to meet:            " << T << " s\n";

    // 2) Illustrative Launch Effort
    const double v0_mps = v0_mis * MILES_TO_M;
    const double a_mps2 = (ACCEL_LEN_M > 0.0) ? (v0_mps * v0_mps) / (2.0 * ACCEL_LEN_M) : 0.0;
    const double F_newton = MASS_KG * a_mps2;
    const double J_Ns     = MASS_KG * v0_mps;

    cout << "\n--- Illustrative Launch Effort ---\n";
    cout << "Assumed mass:            " << MASS_KG << " kg\n";
    cout << "Accel distance:          " << ACCEL_LEN_M << " m\n";
    cout << "Avg acceleration:        " << a_mps2 << " m/s^2\n";
    cout << "Avg force:               " << F_newton << " N\n";
    cout << "Impulse (m*v):           " << J_Ns << " N·s\n";

    // 3) Physics formulas used
    cout << "\n--- Physics formulas used ---\n";
    cout << "1) Projectile motion (no drag):  x(t)=v0 cosθ t,  y(t)=v0 sinθ t - ½ g t²\n";
    cout << "2) Target uniform motion (East):  x(t)=x0+v_T t,   y(t)=y0\n";
    cout << "3) Interception at T=10 s:        v0 cosθ = x_T(T)/T,  v0 sinθ = (y_T(T)+½ g T²)/T,\n";
    cout << "                                   v0 = sqrt((v0 cosθ)² + (v0 sinθ)²)\n";
    cout << "4) Unit conversion:               v(mi/s)=v(mph)/3600\n";
    cout << "5) (Illustrative) launch effort:  a=v²/(2s),  F=ma,  J=mv,  KE/m=½ v²\n";

    // 4) Final Frame (draw last) with vertical exaggeration in plotting only
    auto toPlot = [](const State& s){ return State{ s.x, s.y * Y_STRETCH }; };
    double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;
    auto upd = [&](const State& sP){
        auto s = toPlot(sP);
        xmin = std::min(xmin, s.x); xmax = std::max(xmax, s.x);
        ymin = std::min(ymin, s.y); ymax = std::max(ymax, s.y);
    };
    for (auto& s : proj) upd(s);
    for (auto& s : targ) upd(s);

    double mx = (xmax - xmin) * 0.1 + 0.5;
    double my = (ymax - ymin) * 0.1 + 0.5;
    xmin -= mx; xmax += mx;
    ymin = std::min(0.0, ymin - my);
    ymax += my;

    // Canvas
    const int W = 60, H = 18;

    auto toScreen = [&](const State& sWorld){
        State s = toPlot(sWorld);
        int col = (int)llround((s.x - xmin) / std::max(1e-12, (xmax - xmin)) * (W - 1));
        int row = (int)llround((1.0 - (s.y - ymin) / std::max(1e-12, (ymax - ymin))) * (H - 1));
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
    auto putText = [&](int r, int c, const std::string& s, int p){
        for (size_t i=0;i<s.size();++i) {
            int cc = c + (int)i;
            if (r>=0 && r<H && cc>=0 && cc<W) {
                if (p >= prio[r][cc]) { grid[r][cc] = s[i]; prio[r][cc] = p; }
            }
        }
    };
    enum {P_GROUND=0, P_PPATH=1, P_TPATH=2, P_POEND=3, P_TEND=4}; // low → high

    // ground '_'
    for (int cc = 0; cc < W; ++cc) {
        double x = xmin + (xmax - xmin) * (double)cc / (W - 1);
        auto s0 = toScreen({x, 0.0});
        put(s0.first, cc, '_', P_GROUND);
    }
    // target path '-'
    for (const auto& s : targ) {
        auto p = toScreen(s);
        put(p.first, p.second, '-', P_TPATH);
    }
    // projectile path '.' — only one per row
    std::vector<bool> row_used(H, false);
    for (size_t j = 0; j < proj.size(); ++j) {
        auto p = toScreen(proj[j]);
        if (!row_used[p.first]) {
            put(p.first, p.second, '.', P_PPATH);
            row_used[p.first] = true;
        }
    }
    // target start/end 'X' (highest priority)
    auto Tstart = toScreen(targ.front());   // initial
    auto Tend   = toScreen(targ.back());    // final

    // If both map to same cell, nudge the end marker by ±1 column so both are visible.
    if (Tstart == Tend) {
        if (Tend.second + 1 < W) Tend.second += 1;
        else if (Tend.second - 1 >= 0) Tend.second -= 1;
        // draw a tiny connector if they’re adjacent on the same row
        if (Tstart.first == Tend.first) {
            int lo = std::min(Tstart.second, Tend.second);
            int hi = std::max(Tstart.second, Tend.second);
            for (int c = lo+1; c < hi; ++c) put(Tstart.first, c, '-', P_TPATH);
        }
    }

    put(Tstart.first, Tstart.second, 'X', P_TEND);
    put(Tend.first,   Tend.second,   'X', P_TEND);

    // On-grid labels so you can’t miss them
    const std::string startLabel = "[Start]";
    const std::string endLabel   = "[End]";
    int sCol = (Tstart.second + 1 + (int)startLabel.size() < W)
               ? Tstart.second + 1
               : std::max(0, Tstart.second - 1 - (int)startLabel.size());
    int eCol = (Tend.second + 1 + (int)endLabel.size() < W)
               ? Tend.second + 1
               : std::max(0, Tend.second - 1 - (int)endLabel.size());
    putText(Tstart.first, sCol, startLabel, P_TEND);
    putText(Tend.first,   eCol, endLabel,   P_TEND);

    // projectile start/end 'O'
    auto Pstart = toScreen(proj.front());
    auto Pend   = toScreen(proj.back());
    put(Pstart.first, Pstart.second, 'O', P_POEND);
    put(Pend.first,   Pend.second,   'O', P_POEND);

    cout << "\n--- Final Frame (t = 10 s) ---\n";
    cout << "Legend: _ ground, - target path, X target start/end, . projectile path, O projectile start/end\n";
    cout << "(visual vertical exaggeration: " << Y_STRETCH << "x)\n\n";
    for (auto& line : grid) cout << line << "\n";

    cout << "\n";
    return 0;
}

