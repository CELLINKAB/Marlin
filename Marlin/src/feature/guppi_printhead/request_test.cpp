#include "request.h"
#include "test.h"

using namespace printhead;

namespace test
{

void status_fixed_size()
{
    static_assert_eq(sizeof(Status), 2);
}

void status_raw_conversions()
{
    static constexpr uint16_t raw = 0b0000'0000'1011'0100;
    static constexpr Status status(raw);
    static_assert(status.enable);
    static_assert(status.is_homing);
    static_assert_eq(raw, status.to_raw());
}

}