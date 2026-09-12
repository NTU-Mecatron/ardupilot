/*
   Underwater fins mixing for torpedo-shaped AUVs (ArduTorp)
*/
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/LogStructure.h>

#define TORP_FINS_MAX 6

class FinsMixing {
public:
    enum torp_fins_config {
        PLUS_FIN = 0,
        X_FIN = 1,
        PLUS_FIN_CANARDS = 2,
        X_FIN_CANARDS = 3,
        CUSTOM = 4,
    };

    struct PACKED log_FINS {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float fin1;
        float fin2;
        float fin3;
        float fin4;
        float fin5;
        float fin6;
    };

    FinsMixing();

    /* Do not allow copies */
    CLASS_NO_COPY(FinsMixing);

    void init();
    void setup_fins(torp_fins_config config);
    void output(float roll, float pitch, float yaw);
    void disable_fin(int fin_index);
    void log();

    bool enabled() const { return _config.get() >= 0 && _config.get() <= CUSTOM; }
    int8_t get_config() const { return _config.get(); }

    static const struct AP_Param::GroupInfo var_info[];

private:
    void _compute_control_alloc_mat();

    AP_Int8 _config;
    AP_Float _rll_ft;
    AP_Float _c_rll_ft;
    AP_Float _c_pit_ft;

    uint8_t _num_fins;
    uint8_t _fin_servo_idx[TORP_FINS_MAX];
    bool _fin_disabled[TORP_FINS_MAX];

    float _control_eff_mat[3][TORP_FINS_MAX];
    float _control_alloc_mat[TORP_FINS_MAX][3];
};
