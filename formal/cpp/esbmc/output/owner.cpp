// SPDX-License-Identifier: BSD-3-Clause

#include "frames.hpp"
using namespace output_proof;

void verify_output_initialization()
{
    Output owner{profile::output_parameters};
    check_parameters(owner);
    zero_sample(owner.m_rebase.current);
    zero_vector(owner.m_rebase.error.attitude_b);
    zero_vector(owner.m_rebase.error.velocity_n);
    zero_vector(owner.m_rebase.error.position_n);
    zero_vector(owner.m_rebase.velocity_sum);
    zero_vector(owner.m_rebase.position_sum);
    empty_owner(owner);
    __ESBMC_assert(!owner.m_rebase.prepared, "Initialization/reset cancels staged rebase");
    __ESBMC_assert(owner.m_history.m_length == 0 &&
                       same_configuration(owner.m_configuration, formal_eskf::runtime::TimingConfiguration{}),
                   "Output constructor retains unconfigured history and default timing");
}

void verify_output_session_reset(Output owner)
{
    output_proof::bind_parameters(owner);
    auto const length = owner.m_history.m_length;
    auto const configuration = owner.m_configuration;
    output_frames::owner = &owner;
    output_frames::capture_rebase();
    output_frames::rebase_prepared = false;
    owner.reset();
    output_frames::check_rebase();
    empty_owner(owner);
    __ESBMC_assert(!owner.m_rebase.prepared, "Initialization/reset cancels staged rebase");
    __ESBMC_assert(owner.m_history.m_length == length && same_configuration(owner.m_configuration, configuration),
                   "Output session reset preserves history length and complete configuration");
    output_proof::check_parameters(owner);
}
