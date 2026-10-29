#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Sub : public RC_Channel
{

public:

protected:

    void mode_switch_changed(modeswitch_pos_t new_pos) override;
    bool do_aux_function(aux_func_t ch_option, AuxSwitchPos) override;

private:

    // the first switch positions read from the receiver are treated specially,
    // see mode_switch_changed() and do_aux_function()
    bool mode_switch_seen;
    bool arm_switch_seen;

};

class RC_Channels_Sub : public RC_Channels
{
public:

    RC_Channel_Sub obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Sub *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    // tell the gimbal code all is good with RC input:
    bool in_rc_failsafe() const override { return false; };

    // true while frames are arriving from a directly attached receiver
    bool has_valid_input() const override;

protected:

    // note that these callbacks are not presently used on Plane:
    int8_t flight_mode_channel_number() const override;

};
