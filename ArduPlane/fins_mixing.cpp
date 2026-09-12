/*
   Underwater fins mixing for torpedo-shaped AUVs (ArduTorp)
*/

#include "Plane.h"
#include "fins_mixing.h"

const AP_Param::GroupInfo FinsMixing::var_info[] = {
    // @Param: CONFIG
    // @DisplayName: Underwater fin configuration
    // @Description: Configuration of control fins for underwater torpedo vehicle
    // @Values: 0:Plus-fin, 1:X-fin, 2:Plus-fin with canards, 3:X-fin with canards, 4:Custom
    // @User: Standard
    AP_GROUPINFO("CONFIG", 1, FinsMixing, _config, (int8_t)X_FIN),

    // @Param: RLL_FT
    // @DisplayName: Fin roll factor
    // @Description: Roll effectiveness factor in control effectiveness matrix. Should be equal to r/x where r is the body radius and x is distance from CG to fin along the forward axis.
    // @User: Standard
    AP_GROUPINFO("RLL_FT", 2, FinsMixing, _rll_ft, 0.33f),

    // @Param: C_RLL_FT
    // @DisplayName: Canard roll factor
    // @Description: Roll effectiveness factor of canards in matrix
    // @User: Standard
    AP_GROUPINFO("C_RLL_FT", 3, FinsMixing, _c_rll_ft, 0.0f),

    // @Param: C_PIT_FT
    // @DisplayName: Canard pitch factor
    // @Description: Pitch effectiveness factor of canards in matrix
    // @User: Standard
    AP_GROUPINFO("C_PIT_FT", 4, FinsMixing, _c_pit_ft, 0.0f),

    AP_GROUPEND
};

FinsMixing::FinsMixing()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void FinsMixing::init()
{
    setup_fins((torp_fins_config)_config.get());
}

/*
  Assign control effectiveness matrix (3 x N).
  Precompute and cache control allocation matrix (N x 3).
 */
void FinsMixing::setup_fins(torp_fins_config config)
{
    memset(_control_eff_mat, 0, sizeof(_control_eff_mat));
    memset(_control_alloc_mat, 0, sizeof(_control_alloc_mat));
    memset(_fin_disabled, 0, sizeof(_fin_disabled));

    const float rll = MAX(_rll_ft.get(), 0.5f);

    switch (config) {
    case PLUS_FIN: {
        // Plus fin (1: top, 2: right, 3: bottom, 4: left)
        _num_fins = 4;
        for (uint8_t i = 0; i < _num_fins; i++) {
            _fin_servo_idx[i] = i;
        }

        // Roll row: [ RLL_FT, RLL_FT, RLL_FT, RLL_FT ]
        _control_eff_mat[0][0] = rll;
        _control_eff_mat[0][1] = rll;
        _control_eff_mat[0][2] = rll;
        _control_eff_mat[0][3] = rll;

        // Pitch row: [ 0, 1, 0, -1 ]
        _control_eff_mat[1][0] = 0.0f;
        _control_eff_mat[1][1] = 1.0f;
        _control_eff_mat[1][2] = 0.0f;
        _control_eff_mat[1][3] = -1.0f;

        // Yaw row: [ -1, 0, 1, 0 ]
        _control_eff_mat[2][0] = -1.0f;
        _control_eff_mat[2][1] = 0.0f;
        _control_eff_mat[2][2] = 1.0f;
        _control_eff_mat[2][3] = 0.0f;
        break;
    }

    case X_FIN: {
        // X fin (1: top right, 2: bottom right, 3: bottom left, 4: top left)
        _num_fins = 4;
        for (uint8_t i = 0; i < _num_fins; i++) {
            _fin_servo_idx[i] = i;
        }
        const float c45 = 0.70710678f;

        // Roll row: [ RLL_FT, RLL_FT, RLL_FT, RLL_FT ]
        _control_eff_mat[0][0] = rll;
        _control_eff_mat[0][1] = rll;
        _control_eff_mat[0][2] = rll;
        _control_eff_mat[0][3] = rll;

        // Pitch row: [ 0.707, 0.707, -0.707, -0.707 ]
        _control_eff_mat[1][0] = c45;
        _control_eff_mat[1][1] = c45;
        _control_eff_mat[1][2] = -c45;
        _control_eff_mat[1][3] = -c45;

        // Yaw row: [ -0.707, 0.707, 0.707, -0.707 ]
        _control_eff_mat[2][0] = -c45;
        _control_eff_mat[2][1] = c45;
        _control_eff_mat[2][2] = c45;
        _control_eff_mat[2][3] = -c45;
        break;
    }

    case PLUS_FIN_CANARDS: {
        // Placeholder for Plus-fin with canards
        _num_fins = 6;
        for (uint8_t i = 0; i < _num_fins; i++) {
            _fin_servo_idx[i] = i;
        }
        break;
    }

    case X_FIN_CANARDS: {
        // Placeholder for X-fin with canards
        _num_fins = 6;
        for (uint8_t i = 0; i < _num_fins; i++) {
            _fin_servo_idx[i] = i;
        }
        break;
    }

    case CUSTOM: {
        // Placeholder for custom fin configuration
        _num_fins = 4;
        for (uint8_t i = 0; i < _num_fins; i++) {
            _fin_servo_idx[i] = i;
        }
        break;
    }
    }

    _compute_control_alloc_mat();
}

/*
  Compute and cache the control allocation matrix (N x 3) using the Moore-Penrose pseudoinverse of the control effectiveness matrix (3 x N).
 */
void FinsMixing::_compute_control_alloc_mat()
{
    memset(_control_alloc_mat, 0, sizeof(_control_alloc_mat));

    if (_num_fins == 0) {
        return;
    }

    if (_num_fins == 1) {
        const Vector3f b0(_control_eff_mat[0][0], _control_eff_mat[1][0], _control_eff_mat[2][0]);
        const float dot = b0 * b0;
        if (dot > 1e-6f) {
            const Vector3f alloc_row0 = b0 / dot;
            _control_alloc_mat[0][0] = alloc_row0.x;
            _control_alloc_mat[0][1] = alloc_row0.y;
            _control_alloc_mat[0][2] = alloc_row0.z;
        }
        return;
    }

    // Left inverse: A+ = (A^T * A)^-1 * A^T when m >= n (i.e. 2 fins)
    if (_num_fins == 2) {
        const Vector3f b0(_control_eff_mat[0][0], _control_eff_mat[1][0], _control_eff_mat[2][0]);
        const Vector3f b1(_control_eff_mat[0][1], _control_eff_mat[1][1], _control_eff_mat[2][1]);
        const float K[4] = {
            b0 * b0, b0 * b1,
            b1 * b0, b1 * b1
        };
        float K_inv[4];
        if (mat_inverse(K, K_inv, 2)) {
            const Vector3f alloc_row0 = b0 * K_inv[0] + b1 * K_inv[1];
            const Vector3f alloc_row1 = b0 * K_inv[2] + b1 * K_inv[3];

            _control_alloc_mat[0][0] = alloc_row0.x;
            _control_alloc_mat[0][1] = alloc_row0.y;
            _control_alloc_mat[0][2] = alloc_row0.z;

            _control_alloc_mat[1][0] = alloc_row1.x;
            _control_alloc_mat[1][1] = alloc_row1.y;
            _control_alloc_mat[1][2] = alloc_row1.z;
        }
        return;
    }

    // Right inverse: A+ = A^T * (A * A^T)^-1 (when m <= n, i.e. >= 3 fins)
    // Compute M = B * B^T (3x3) using outer product sum: sum(b_k * b_k^T)
    Matrix3f M;
    for (uint8_t k = 0; k < _num_fins; k++) {
        const Vector3f b_k(_control_eff_mat[0][k], _control_eff_mat[1][k], _control_eff_mat[2][k]);
        M += b_k.mul_rowcol(b_k);
    }

    // Invert M (guaranteed non-singular since roll factor >= 0.5)
    if (!M.invert()) {
        return;
    }

    // Compute B^dagger = B^T * M^-1
    // Each row of B^dagger for fin k is b_k^T * M^-1 = b_k.row_times_mat(M)
    for (uint8_t k = 0; k < _num_fins; k++) {
        const Vector3f b_k(_control_eff_mat[0][k], _control_eff_mat[1][k], _control_eff_mat[2][k]);
        const Vector3f alloc_row = b_k.row_times_mat(M);
        _control_alloc_mat[k][0] = alloc_row.x;
        _control_alloc_mat[k][1] = alloc_row.y;
        _control_alloc_mat[k][2] = alloc_row.z;
    }
}

/*
  Remove the corresponding column in the control effectiveness matrix and recompute the control allocation matrix.
  Used for fin failure handling.
 */
void FinsMixing::disable_fin(int fin_index)
{
    if (fin_index < 0 || fin_index >= _num_fins || _num_fins <= 1) {
        return;
    }

    // Mark physical servo as disabled so its output is zeroed
    const uint8_t disabled_servo = _fin_servo_idx[fin_index];
    if (disabled_servo < TORP_FINS_MAX) {
        _fin_disabled[disabled_servo] = true;
    }

    // Shift columns of _control_eff_mat and _fin_servo_idx left
    for (uint8_t k = fin_index; k < _num_fins - 1; k++) {
        _fin_servo_idx[k] = _fin_servo_idx[k + 1];
        for (uint8_t r = 0; r < 3; r++) {
            _control_eff_mat[r][k] = _control_eff_mat[r][k + 1];
        }
    }

    _num_fins--;

    // Recompute pseudoinverse with (N - 1) fins
    _compute_control_alloc_mat();
}

/*
  Compute and apply the fin deflections based on the desired roll, pitch, and yaw commands.
  Disabled fins are set to zero output.
  Fins must be assigned script functions 1 to 6.
 */
void FinsMixing::output(float roll, float pitch, float yaw)
{
    if (!enabled()) {
        return;
    }

    // Zero output for any disabled fins
    for (uint8_t s = 0; s < TORP_FINS_MAX; s++) {
        if (_fin_disabled[s]) {
            SRV_Channels::set_output_scaled((SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_scripting1 + s), 0.0f);
        }
    }

    // Allocate commands to active fins: delta = B^dagger * [roll, pitch, yaw]^T
    const Vector3f cmd(roll, pitch, yaw);
    for (uint8_t i = 0; i < _num_fins; i++) {
        const uint8_t servo_idx = _fin_servo_idx[i];
        if (servo_idx >= TORP_FINS_MAX) {
            continue;
        }

        const Vector3f alloc_row(_control_alloc_mat[i][0], _control_alloc_mat[i][1], _control_alloc_mat[i][2]);
        float deflection = alloc_row * cmd;

        // Constrain to standard servo range [-3000, 3000] centidegrees
        deflection = constrain_float(deflection, -3000.0f, 3000.0f);

        SRV_Channels::set_output_scaled(
            (SRV_Channel::Aux_servo_function_t)(SRV_Channel::k_scripting1 + servo_idx),
            deflection
        );
    }
}

