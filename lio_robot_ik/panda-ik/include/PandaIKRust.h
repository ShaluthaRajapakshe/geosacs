#include <stdint.h>
extern "C" {
    bool init(const char* urdf);
    void solve(double* robot_state, const char* transform_name, const double* goal_x, const double* goal_q, const double* velocity, bool* errors, double w);
}