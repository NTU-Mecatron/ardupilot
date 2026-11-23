#include "Sub.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Sub
#define RC_CHANNEL_SUBCLASS RC_Channel_Sub

#include <RC_Channel/RC_Channels_VarInfo.h>

// note that this callback is not presently used on Plane:
int8_t RC_Channels_Sub::flight_mode_channel_number() const
{
    return sub.g2.flight_mode_chan.get();
}

bool RC_Channels_Sub::has_valid_input() const
{
    // joystick input arrives as overrides and is not counted here
    return has_had_rc_receiver() && !sub.failsafe.rc_input;
}

void RC_Channel_Sub::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos >= ARRAY_SIZE(sub.g2.flight_modes)) {
        // should not have been called
        return;
    }

    // the first position is only applied while disarmed, so switching the
    // radio on mid-mission does not take control from the companion computer
    const bool first_reading = !mode_switch_seen;
    mode_switch_seen = true;
    if (first_reading && sub.motors.armed()) {
        return;
    }

    sub.set_mode((Mode::Number)sub.g2.flight_modes[new_pos].get(), ModeReason::RC_COMMAND);
}

bool RC_Channel_Sub::do_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    if (ch_option == AUX_FUNC::ARMDISARM && !arm_switch_seen) {
        // the first position is only recorded, so a switch left in the arm
        // position at boot does not arm the vehicle
        arm_switch_seen = true;
        if (ch_flag == AuxSwitchPos::HIGH) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Arm switch is high, toggle it to arm");
        }
        return true;
    }

    return RC_Channel::do_aux_function(ch_option, ch_flag);
}
