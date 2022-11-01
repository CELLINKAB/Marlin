// Copyright Cellink 2022 - Licensed under GPLv3

#pragma once

#include "../inc/MarlinConfig.h"
#include "../module/planner.h"

#include "simple_TMC_controller.h"

// fully generic probe interface idea
// crtp class with a stow/deploy impl and a read impl
// read impl is either DigitalProbe<PIN> or AnalogProbe<Pin>
// probe types inherit from Probe<StowDeploy, ProbeType>

#define SRP_DEPLOY_VELOCITY 36000
#define SRP_STOW_VELOCITY -64000
#define SRP_STALL_THRESHOLD 120
#define SRP_STEPPER_CURRENT 350
#define SRP_RETRACT_TIME 9000

struct StepperRetractingProbe
{
    struct Config
    {
        int32_t deploy_velocity;
        int32_t stow_velocity;
        int16_t stall_threshold;
        uint16_t stepper_current;
        uint32_t minimum_retract_time;
    };

    StepperRetractingProbe()
        : config{SRP_DEPLOY_VELOCITY,
                 SRP_STOW_VELOCITY,
                 SRP_STALL_THRESHOLD,
                 SRP_STEPPER_CURRENT,
                 SRP_RETRACT_TIME}
        , stepper{STMC::init(
              SimpleTMCConfig(PROBE_SERIAL_ADDRESS, config.stall_threshold, config.stepper_current))}
    {}

    void deploy()
    {
        switch (state) {
        case ProbeState::Deployed:
            [[fallthrough]];
        case ProbeState::Unknown:
            stepper.raw_move(config.stow_velocity);
            delay(200);
            stepper.stop();
            [[fallthrough]];
        case ProbeState::Stowed:
            stepper.blocking_move_until_stall(config.deploy_velocity, config.minimum_retract_time * 2);
            state = ProbeState::Deployed;
        }
    }

    void stow()
    {
        if (state != ProbeState::Deployed) {
            deploy();
        }
        stepper.raw_move(config.stow_velocity);
        delay(config.minimum_retract_time);
        stepper.stop();
        state = ProbeState::Stowed;
    }

    const Config& get_config() const { return config; }

    void set_config(const Config& conf)
    {
        // load config from flash
        stepper.rms_current(conf.stepper_current);
        stepper.stall_threshold(conf.stall_threshold);
        config = conf;
    }

    void report_config(bool for_replay) const
    {
        if (for_replay) {
            SERIAL_ECHOLNPGM_P("M1029 T",
                               config.stall_threshold,
                               " C",
                               config.stepper_current,
                               " S",
                               config.stow_velocity,
                               " D",
                               config.deploy_velocity,
                               " M",
                               config.minimum_retract_time);
        } else {
            SERIAL_ECHOLNPGM_P("Stall threshold: ",
                               config.stall_threshold,
                               "\nStepper current: ",
                               config.stepper_current,
                               "\nStow velocity: ",
                               config.stow_velocity,
                               "\nDeploy velocity: ",
                               config.deploy_velocity,
                               "\nBackoff time: ",
                               config.minimum_retract_time);
        }
    }

    inline void reset_position() {state = ProbeState::Unknown;}

private:
    using STMC = SimpleTMC<PROBE_EN_PIN, PROBE_STOP_PIN>;

    Config config;

    enum class ProbeState {
        Unknown,
        Stowed,
        Deployed
    } state = ProbeState::Unknown;

    STMC::type stepper;
};

extern StepperRetractingProbe stepper_probe;