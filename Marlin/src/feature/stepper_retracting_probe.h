// Copyright Cellink 2022 - Licensed under GPLv3

#pragma once

#include "../inc/MarlinConfig.h"
#include "../module/planner.h"

#include <optional>

#include "simple_TMC_controller.h"

// fully generic probe interface idea
// crtp class with a stow/deploy impl and a read impl
// read impl is either DigitalProbe<PIN> or AnalogProbe<Pin>
// probe types inherit from Probe<StowDeploy, ProbeType>

struct StepperRetractingProbe
{
    static constexpr int32_t SRP_DEPLOY_VELOCITY = -100'000;
    static constexpr int32_t SRP_STOW_VELOCITY = 100'000;
    static constexpr uint8_t SRP_STALL_THRESHOLD = 120;
    static constexpr uint32_t SRP_STEPPER_CURRENT = 1000;
    static constexpr uint32_t SRP_RETRACT_TIME = 7000;
    static constexpr float SRP_STEPPER_RSENSE = 0.11f;

    struct Config
    {
        int32_t deploy_velocity;
        int32_t stow_velocity;
        int16_t stall_threshold;
        uint16_t stepper_current;
        uint32_t minimum_retract_time;
    };

    constexpr StepperRetractingProbe()
        : config{SRP_DEPLOY_VELOCITY,
                 SRP_STOW_VELOCITY,
                 SRP_STALL_THRESHOLD,
                 SRP_STEPPER_CURRENT,
                 SRP_RETRACT_TIME}
        , state{ProbeState::Unknown}
        , _stepper{}
    {}

    void deploy();

    void stow();

    constexpr const Config& get_config() const { return config; }

    inline void set_config(const Config& conf)
    {
        config = conf;
        // reset stepper to force reinitialization next use
        _stepper.reset();
    }

    void report_config(bool for_replay) const;

    constexpr inline void reset_position() noexcept { state = ProbeState::Unknown; }

    constexpr inline bool is_deployed() const noexcept {return state == ProbeState::Deployed;}

private:
    using STMC = SimpleTMC<PROBE_EN_PIN, PROBE_STOP_PIN>;

    Config config;

    enum class ProbeState {
        Unknown,
        Stowed,
        Deployed
    } state;

    std::optional<STMC> _stepper;

    void stepper_init();

    inline STMC& stepper()
    {
        if (!_stepper.has_value())
            stepper_init();
        return *_stepper;
    }
};

extern StepperRetractingProbe stepper_probe;