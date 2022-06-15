// Copyright Cellink 2022 - Licensed under GPLv3

#pragma once

#include "../inc/MarlinConfig.h"
#include "../module/planner.h"

#include "simple_TMC_controller.h"

// fully generic probe interface idea
// crtp class with a stow/deploy impl and a read impl
// read impl is either DigitalProbe<PIN> or AnalogProbe<Pin>
// probe types inherit from Probe<StowDeploy, ProbeType>

struct StepperRetractingProbe
{
    void deploy()
    {
        switch (state) {
        case ProbeState::Deployed:
            [[fallthrough]];
        case ProbeState::Unknown:
            stepper.raw_move(STOW_VELOCITY);
            delay(200);
            stepper.stop();
            [[fallthrough]];
        case ProbeState::Stowed:
            stepper.blocking_move_until_stall(DEPLOY_VELOCITY);
            state = ProbeState::Deployed;
        }
    }

    void stow()
    {
        if (state != ProbeState::Deployed) {
            deploy();
        }
        stepper.raw_move(STOW_VELOCITY);
        delay(STOW_TIME);
        stepper.stop();
        state = ProbeState::Stowed;
    }

private:
    constexpr static int32_t DEPLOY_VELOCITY = 38000;
    constexpr static int32_t STOW_VELOCITY = -65000;
    constexpr static uint8_t HW_ADDRESS = 2;
    constexpr static uint8_t STALL_THRESHOLD = 55;
    constexpr static uint32_t MOTOR_CURRENT = 225;
    constexpr static uint32_t STOW_TIME = 7000;

    enum class ProbeState
    {
        Unknown,
        Stowed,
        Deployed
    } state = ProbeState::Unknown;

    using STMC = SimpleTMC<PROBE_EN_PIN, PROBE_STOP_PIN>;
    STMC::type stepper{STMC::init(SimpleTMCConfig(HW_ADDRESS, STALL_THRESHOLD, MOTOR_CURRENT))};
};

extern StepperRetractingProbe stepper_probe;