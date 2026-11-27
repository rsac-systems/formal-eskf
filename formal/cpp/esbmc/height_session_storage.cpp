/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "height_state_support.hpp"
#ifndef FORMAL_ESKF_PROOF_HEIGHT_SOURCE
#define FORMAL_ESKF_PROOF_HEIGHT_SOURCE 0
#endif
static_assert(FORMAL_ESKF_PROOF_HEIGHT_SOURCE < 2);
using height_proof::same_session;
using height_proof::Session;
using Sessions = std::FormalEskfHeightSessions<Session, 2U>;

void verify_height_session_storage(Session first, Session second, height_proof::Manager::DiagnosticUpdate first_event,
                                   height_proof::Manager::DiagnosticUpdate second_event)
{
    auto const old_first = first;
    auto const old_second = second;
    Sessions values;
    values[0U] = first;
    values[1U] = second;
    Sessions copy(values);
    Sessions assigned;
    assigned = values;
    assigned = assigned;
    __ESBMC_assert(
        same_session(copy[0U], first) && same_session(copy[1U], second) && same_session(assigned[0U], first) &&
            same_session(assigned[1U], second),
        "height session storage: copy construction, assignment and self-assignment retain every element field");

    constexpr std::size_t index = FORMAL_ESKF_PROOF_HEIGHT_SOURCE;
    auto & selected = assigned[index];
    auto & first_element = assigned[0U];
    auto & second_element = assigned[1U];
    Session * expected = &first_element;
    if constexpr (index == 1U)
    {
        expected = &second_element;
    }

    __ESBMC_assert(&selected == expected, "height session storage: indexing retains element identity");

    selected.active = !selected.active;
    auto expected_first = first;
    auto expected_second = second;
    if constexpr (index == 0U)
    {
        expected_first.active = !expected_first.active;
    }
    else
    {
        expected_second.active = !expected_second.active;
    }

    __ESBMC_assert(same_session(assigned[0U], expected_first) && same_session(assigned[1U], expected_second) &&
                       same_session(values[0U], first) && same_session(values[1U], second) &&
                       same_session(copy[0U], first) && same_session(copy[1U], second),
                   "height session storage: one element write cannot alias another element or any copied container");

    unsigned count = 0U;
    for (auto & element : copy)
    {
        bool valid = false;
        if (count == 0U)
        {
            valid = same_session(element, first);
        }
        else if (count == 1U)
        {
            valid = same_session(element, second);
        }

        __ESBMC_assert(valid, "height session storage: iteration visits the two elements in order");

        ++count;
    }

    __ESBMC_assert(count == 2U, "height session storage: iteration terminates at two elements");

    Sessions const & constant = values;
    count = 0U;
    for (auto const & element : constant)
    {
        bool valid = false;
        if (count == 0U)
        {
            valid = same_session(element, first);
        }
        else if (count == 1U)
        {
            valid = same_session(element, second);
        }

        __ESBMC_assert(valid, "height session storage: const iteration retains every element field");

        ++count;
    }

    __ESBMC_assert(count == 2U && same_session(first, old_first) && same_session(second, old_second),
                   "height session storage: const traversal and all copies preserve their original inputs");

    using Event = height_proof::Manager::DiagnosticUpdate;
    using Events = std::FormalEskfHeightSessions<Event, 2U>;
    Events events{};
    __ESBMC_assert(height_proof::same_diagnostic(events[0], Event{}) &&
                       height_proof::same_diagnostic(events[1], Event{}),
                   "height diagnostic storage: both empty event records have all fields initialized");

    events[0] = first_event;
    events[1] = second_event;
    Events copied_events(events);
    Events assigned_events;
    assigned_events = copied_events;
    assigned_events = assigned_events;
    __ESBMC_assert(height_proof::same_diagnostic(assigned_events[0], first_event) &&
                       height_proof::same_diagnostic(assigned_events[1], second_event),
                   "height diagnostic storage: complete event copies and self assignment");

    assigned_events[index].reset = !assigned_events[index].reset;
    auto expected_event0 = first_event;
    auto expected_event1 = second_event;
    if constexpr (index == 0)
    {
        expected_event0.reset = !expected_event0.reset;
    }
    else
    {
        expected_event1.reset = !expected_event1.reset;
    }

    __ESBMC_assert(height_proof::same_diagnostic(assigned_events[0], expected_event0) &&
                       height_proof::same_diagnostic(assigned_events[1], expected_event1) &&
                       height_proof::same_diagnostic(events[0], first_event) &&
                       height_proof::same_diagnostic(events[1], second_event),
                   "height diagnostic storage: selected record mutation preserves the other source and originals");
}

int main() { return 0; }
