// Copyright Cellink 2022 - Licensed under GPLv3

#pragma once

#include "../inc/MarlinConfig.h"
#include "../module/planner.h"
#include "simple_TMC_controller.h"
#include "analog_surface_probe.h"

struct RetractingDisplacementProbe : AnalogProbe<RetractingDisplacementProbe, PROBE_READ_PIN>
{
    RetractingDisplacementProbe() : AnalogProbe_t([this]{SERIAL_ECHOLNPGM("Probe:", get_distance_raw());}) {}

    bool stow_impl()
    {
        return stow_deploy(ProbeCommand::Stow);
    }
    
    bool deploy_impl()
    {
        return stow_deploy(ProbeCommand::Deploy);
    }

    bool probe_to_z_impl(const float z_max, const float fr_mm_s) {
        static constexpr uint32_t RAW_THRESHOLD = 400;
        static constexpr float MOVE_INCREMENT = 0.5f;
        static constexpr float SANITY_CHECK_THRESHOLD = 1.2f;
        auto z_pos = planner.get_axis_position_mm(AxisEnum::Z_AXIS);
        //planner.buffer_segment(pos, fr_mm_s);
        auto d = get_distance_raw();
        while (d > RAW_THRESHOLD && z_pos > z_max) {
            z_pos -= MOVE_INCREMENT;
            do_blocking_move_to_z(z_pos, fr_mm_s);
            d = get_distance_raw();
        } 
        if DEBUGGING(LEVELING) SERIAL_ECHOLNPGM("probe value when triggered:", d);
        if (get_distance_avg() > SANITY_CHECK_THRESHOLD)
            return true;
        return false;
    }

    float probe_val_impl() const {
        static constexpr float PROBE_LENGTH_OFFSET = -5.0f;
        return get_distance_avg() + PROBE_LENGTH_OFFSET;
    }
    
    uint32_t debug_probe(const float x, const float y)
    {
        do_blocking_move_to_z(Z_CLEARANCE_DEPLOY_PROBE);
        do_blocking_move_to_xy(x, y);
        do_blocking_move_to_z(0.0f);
        delay(50); // allow signal to stabilize
        const auto probe_val = analogRead(PROBE_READ_PIN);
        do_blocking_move_to_z(Z_CLEARANCE_DEPLOY_PROBE);
        return probe_val;
    }
    
    float get_distance_impl() const {
        static constexpr float SCALE_FACTOR = 0.00025668f;
        const auto raw_distance = get_distance_raw();
        return (static_cast<float>(raw_distance) * SCALE_FACTOR);
    }

private:
    constexpr static uint32_t RDP_VELOCITY = 49000;
    constexpr static uint8_t HW_ADDRESS = 2;
    constexpr static uint8_t STALL_THRESHOLD = 65;
    constexpr static uint32_t MOTOR_CURRENT = 280;

    enum class ProbeState {
        Unknown,
        Stowed,
        Deployed
    } state = ProbeState::Unknown;

    enum class ProbeCommand {
        Stow,
        Deploy
    };

    /**
     * @brief Helper to backoff the stepper from an endstop
     * 
     * @param velocity intended direction of primary move, gets inverted internally
     */
    void backoff(const int32_t velocity)
    {
        static constexpr uint32_t BACKOFF_DELAY = 100;
        stepper.raw_move(-velocity);
        delay(BACKOFF_DELAY);
        stepper.stop();
    }

    bool stow_deploy(ProbeCommand command) {
        const auto velocity = (command == ProbeCommand::Stow) ? RDP_VELOCITY : -RDP_VELOCITY;
        const bool check_state = (command == ProbeCommand::Stow) ? (state == ProbeState::Stowed) : (state == ProbeState::Deployed);
        if (check_state) return false;
        backoff(velocity);
        stepper.blocking_move_until_stall(velocity);
        state = ProbeState::Stowed;
        return false;
    }

    using STMC = SimpleTMC<RDP_EN_PIN, RDP_STOP_PIN>;
    STMC::type stepper{STMC::init(SimpleTMCConfig(HW_ADDRESS, STALL_THRESHOLD, MOTOR_CURRENT))};
};

extern RetractingDisplacementProbe::AnalogProbe_t& analog_probe;