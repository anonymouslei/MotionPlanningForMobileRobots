#include <hw_tool.h>

#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l,
                               Vector3d global_xyz_u, int max_x_id,
                               int max_y_id, int max_z_id) {
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y,
                          const double coord_z) {
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y,
                             const double coord_z) {
    Vector3d pt;
    Vector3i idx;

    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
            idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i &index) {
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d &pt) {
    Vector3i idx;
    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
        min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
        min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d &coord) {
    return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,
                                Eigen::Vector3d _start_velocity,
                                Eigen::Vector3d _target_position) {
    double optimal_cost =
        100000; // this just to initial the optimal_cost, you can delete it
    /*




    STEP 2: go to the hw_tool.cpp and finish the function
    Homeworktool::OptimalBVP the solving process has been given in the document

    because the final point of trajectory is the start point of OBVP, so we
    input the pos,vel to the OBVP

    after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the
    optimal cost of this trajectory


    */

    auto dp = _target_position - _start_position;
    auto dv = Eigen::Vector3d(0, 0, 0) - _start_velocity;
    auto v0 = _start_velocity;

    double c_3, c_2, c_1, c_0;

    c_3 = 0.;
    c_2 = -1 * (12 * v0(0) * v0(0) + 12 * dv(0) * v0(0) + 4 * dv(0) * dv(0)) -
          1 * (12 * v0(1) * v0(1) + 12 * dv(1) * v0(1) + 4 * dv(1) * dv(1)) -
          1 * (12 * v0(2) * v0(2) + 12 * dv(2) * v0(2) + 4 * dv(2) * dv(2));
    c_1 = 2 * (24 * dp(0) * v0(0) + 12 * dp(0) * dv(0)) +
          2 * (24 * dp(1) * v0(1) + 12 * dp(1) * dv(1)) +
          2 * (24 * dp(2) * v0(2) + 12 * dp(2) * dv(2));
    c_0 = -3 * 12 * dp(0) * dp(0) - 3 * 12 * dp(1) * dp(1) -
          3 * 12 * dp(2) * dp(2);

    // c_2 = -12 * _start_velocity(0) * _start_velocity(0) +
    //       12 * _start_velocity(0) * dv(0) + 4 * dv(0) * dv(0) -
    //       12 * _start_velocity(1) * _start_velocity(1) +
    //       12 * _start_velocity(1) * dv(1) + 4 * dv(1) * dv(1) -
    //       12 * _start_velocity(2) * _start_velocity(2) +
    //       12 * _start_velocity(2) * dv(2) + 4 * dv(2) * dv(2);

    // c_1 = 48 * dp(0) * _start_velocity(0) + 24 * dp(0) * dv(0) +
    //       48 * dp(1) * _start_velocity(1) + 24 * dp(1) * dv(1) +
    //       48 * dp(2) * _start_velocity(2) + 24 * dp(2) * dv(2);

    // c_0 = -36 * dp(0) * dp(0) - 36 * dp(1) * dp(1) - 36 * dp(2) * dp(2);

    Eigen::Matrix4d m(Eigen::Matrix4d::Zero());
    m.block<3, 3>(1, 0) = Eigen::Matrix3d::Identity();
    m(0, 3) = -c_0;
    m(1, 3) = -c_1;
    m(2, 3) = -c_2;
    m(3, 3) = -c_3;

    Eigen::EigenSolver<Eigen::Matrix4d> eigenSolver(m);

    auto evs = eigenSolver.eigenvalues();

    optimal_cost = -1;

    // cout << evs << endl;
    for (int i = 0; i < 4; ++i) {
        if (evs(i).imag() == 0) {
            double T = evs(i).real();
            if (T > 0) {
                // double k;
                // double T4 = T * T * T * T;
                // double T3 = T * T * T;
                // double T2 = T * T;
                // cout << T << endl;
                // k = 1 - 3 / T4 * 12 * dp(0) * dp(0) +
                //     2 / T3 * (24 * dp(0) * v0(0) + 12 * dp(0) * dv(0)) -
                //     1 / T2 *
                //         (12 * v0(0) * v0(0) + 12 * dv(0) * v0(0) +
                //          4 * dv(0) * dv(0)) -
                //     3 / T4 * 12 * dp(1) * dp(1) +
                //     2 / T3 * (24 * dp(1) * v0(1) + 12 * dp(1) * dv(1)) -
                //     1 / T2 *
                //         (12 * v0(1) * v0(1) + 12 * dv(1) * v0(1) +
                //          4 * dv(1) * dv(1)) -
                //     3 / T4 * 12 * dp(2) * dp(2) +
                //     2 / T3 * (24 * dp(2) * v0(2) + 12 * dp(2) * dv(2)) -
                //     1 / T2 *
                //         (12 * v0(2) * v0(2) + 12 * dv(2) * v0(2) +
                //          4 * dv(2) * dv(2));

                auto J = calJ(T, dp, dv, _start_velocity);

                if (optimal_cost == -1 || optimal_cost > J) {
                    optimal_cost = J;
                }
            }
        }
    }
    cout << optimal_cost << endl;
    return optimal_cost;
}

double Homeworktool::calJ(double T, const Eigen::Vector3d &dp,
                          const Eigen::Vector3d &dv,
                          const Eigen::Vector3d &v0) {

    double T3 = T * T * T;
    double T2 = T * T;

    double J;
    J = T +
        (12 / T3 *
             (dp(0) * dp(0) - 2 * dp(0) * v0(0) * T + v0(0) * v0(0) * T2) -
         12 / T2 * (dp(0) * dv(0) - dv(0) * v0(0) * T) +
         4 / T * dv(0) * dv(0)) +
        (12 / T3 *
             (dp(1) * dp(1) - 2 * dp(1) * v0(1) * T + v0(1) * v0(1) * T2) -
         12 / T2 * (dp(1) * dv(1) - dv(1) * v0(1) * T) +
         4 / T * dv(1) * dv(1)) +
        (12 / T3 *
             (dp(2) * dp(2) - 2 * dp(2) * v0(2) * T + v0(2) * v0(2) * T2) -
         12 / T2 * (dp(2) * dv(2) - dv(2) * v0(2) * T) + 4 / T * dv(2) * dv(2));

    return J;
}