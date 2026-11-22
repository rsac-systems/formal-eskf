// SPDX-License-Identifier: BSD-3-Clause

#include "selected_frame.hpp"
namespace
{
using namespace output_boundary;
Sample committed;
} // namespace
namespace formal_eskf::runtime
{
template <> void OutputPredictor<output_proof::Backend, 64>::commit_history()
{
    check_selected();
    __ESBMC_init_object(owner);
    restore_outer();
    restore_selected_workspace();
    m_current = committed;
    capture_selected_history();
}
} // namespace formal_eskf::runtime
#include <source_steps.hpp>
void verify_output_reset_frame(Output object, Sample newest)
{
    output_proof::bind_parameters(object);
    owner = &object;
    metadata(object);
    __ESBMC_assume(object.m_history.m_length >= 2);
    capture();
    capture_selected();
    committed = newest;
    bool const seed_only = object.m_history.m_count == 0;
    (void)output_steps::reset_finish(object, seed_only);
    check_selected();
    output_proof::check_parameters(object);
}
