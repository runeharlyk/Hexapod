#ifndef Kinematics_h
#define Kinematics_h

#include <utils/math_utils.h>
#include <message_types.h>

struct HexapodConfig {
    const float mountX[6];
    const float mountY[6];
    const float mountAngle[6];
    const float rootToJoint1;
    const float joint1ToJoint2;
    const float joint2ToJoint3;
    const float joint3ToTip;
};

constexpr HexapodConfig hexapodConfig = {{44.82, 61.03, 44.82, -44.82, -61.03, -44.82},
                                         {74.82, 0, -74.82, 74.82, 0, -74.82},
                                         {45, 0, -45, -225, -180, -135},
                                         0,
                                         38,
                                         54.06,
                                         97};

constexpr void get_transformation_matrix(const BodyStateMsg &b, float T[4][4]) {
    float co = cosf(b.omega), so = sinf(b.omega);
    float cp = cosf(b.phi), sp = sinf(b.phi);
    float cs = cosf(b.psi), ss = sinf(b.psi);

    T[0][0] = cp * cs;
    T[0][1] = -cp * ss;
    T[0][2] = sp;
    T[0][3] = b.xm;
    T[1][0] = so * sp * cs + ss * co;
    T[1][1] = -so * sp * ss + co * cs;
    T[1][2] = -so * cp;
    T[1][3] = b.ym;
    T[2][0] = so * ss - sp * co * cs;
    T[2][1] = so * cs + sp * ss * co;
    T[2][2] = co * cp;
    T[2][3] = b.zm;
    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;
}

// Per-joint travel in degrees, order coxa, femur, tibia. Mirrors COXA/FEMUR/TIBIA_RANGE in
// simulation/src/resources/build_model.py, which derives them from the servo's real travel.
static constexpr float JOINT_LIMIT_DEG[3] = {31.5f, 90.0f, 149.0f};

class Kinematics {
  private:
    float mountX[6], mountY[6], rootJ1, j1J2, j2J3, j3Tip;
    float ca[6], sa[6], mountPos[6][3];

  public:
    Kinematics(const HexapodConfig &c = hexapodConfig) {
        rootJ1 = c.rootToJoint1;
        j1J2 = c.joint1ToJoint2;
        j2J3 = c.joint2ToJoint3;
        j3Tip = c.joint3ToTip;
        for (int i = 0; i < 6; i++) {
            mountX[i] = c.mountX[i];
            mountY[i] = c.mountY[i];
            float a = c.mountAngle[i] * M_PI / 180;
            ca[i] = std::cos(a);
            sa[i] = std::sin(a);
            mountPos[i][0] = mountX[i];
            mountPos[i][1] = mountY[i];
            mountPos[i][2] = 0;
        }
    }

    void inverseKinematics(const BodyStateMsg &b, float ang[18]) {
        float T[4][4], w[4], dx, dy, radial, vertical, base, lr2, lr, c1, c2;

        get_transformation_matrix(b, T);

        for (int i = 0; i < 6; i++) {
            MAT_MULT(T, b.feet[i], w, 4, 4, 1);

            float wx = w[0] - mountPos[i][0];
            float wy = w[1] - mountPos[i][1];
            float wz = w[2] - mountPos[i][2];

            float lx = wx * ca[i] + wy * sa[i];
            float ly = wx * sa[i] - wy * ca[i];
            float lz = wz;

            dx = lx - rootJ1;
            dy = ly;
            float a0 = -atan2f(dy, dx);

            radial = hypotf(dx, dy) - j1J2;
            vertical = lz;
            base = atan2f(vertical, radial);
            lr2 = radial * radial + vertical * vertical;
            lr = sqrtf(lr2);

            c1 = (lr2 + j2J3 * j2J3 - j3Tip * j3Tip) / (2 * j2J3 * lr);
            c2 = (lr2 - j2J3 * j2J3 + j3Tip * j3Tip) / (2 * j3Tip * lr);
            c1 = fmaxf(-1, fminf(1, c1));
            c2 = fmaxf(-1, fminf(1, c2));

            float a1 = acosf(c1);
            float a2 = acosf(c2);

            ang[i * 3 + 0] = RAD_TO_DEG_F(a0);
            ang[i * 3 + 1] = RAD_TO_DEG_F(base + a1);
            ang[i * 3 + 2] = RAD_TO_DEG_F(-(a1 + a2));
        }
    }

    // True when foot i of b lies inside the leg's reach annulus, which is equivalent up to rounding to
    // the condition under which neither acos argument in inverseKinematics saturates. Mirrors foot_reachable() in
    // simulation/src/robot/animation.py.
    bool footReachable(const BodyStateMsg &b, int i) const {
        float T[4][4], w[4];
        get_transformation_matrix(b, T);
        MAT_MULT(T, b.feet[i], w, 4, 4, 1);
        const float wx = w[0] - mountPos[i][0];
        const float wy = w[1] - mountPos[i][1];
        const float lx = wx * ca[i] + wy * sa[i];
        const float ly = wx * sa[i] - wy * ca[i];
        const float dx = lx - rootJ1;
        const float radial = hypotf(dx, ly) - j1J2;
        const float lr = hypotf(radial, w[2] - mountPos[i][2]);
        return fabsf(j2J3 - j3Tip) <= lr && lr <= j2J3 + j3Tip;
    }
};

#endif