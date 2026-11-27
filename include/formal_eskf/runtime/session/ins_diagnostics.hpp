// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstdint>
#include <formal_eskf/runtime/fusion/measurement_fusion.hpp>

namespace formal_eskf::runtime
{

/** Identity and clocks of the computation, never the latest consumed watermark.
 * session_id is assigned by the caller; zero means unavailable. Frame epoch is
 * session-local, including local heading admission before geographic attachment.
 */
struct EvaluationEpoch
{
    std::uint64_t session_id{}, raw_time_us{}, arrival_time_us{}, selection_time_us{}, evaluation_time_us{};
    std::uint64_t source_generation{}, frame_epoch{}, order{}, interval_us{};
    std::uint32_t device_id{}, reference_id{};
    std::uint8_t calibration_count{};
};

/** Last actual group evaluation plus independent reset evidence and lifetime counts.
 * Residuals use measurement - prediction. Only gate_valid permits reading S/ratios;
 * gravity has no gate. Counts wrap modulo 2^64 and are not reset by source changes.
 */
struct FusionDiagnostic
{
    enum class Outcome
    {
        none,
        checked,
        inhibited,
        rejected,
        corrected,
        failed,
        rolled_back
    };

    struct Counts
    {
        std::uint64_t evaluations{}, attempts{}, corrected{}, rejected{}, failed{}, resets{}, reset_failures{};
    };

    struct Reset
    {
        EvaluationEpoch epoch{};
        Status status = Status::success;
        bool attempted{}, committed{};
    };

    EvaluationEpoch epoch{};
    Outcome outcome = Outcome::none;
    Status status = Status::success;
    Status transaction_status = Status::success;
    bool gate_valid{}, innovation_limited{};
    std::uint8_t dimension{};
    std::array<double, 3> innovation{}, variance{}, test_ratio{}, fusion_innovation{};
    std::uint64_t last_correction_us{}, last_reset_us{};
    Reset reset{};
    Counts counts{};

    void change_source()
    {
        auto const totals = counts;
        *this = {};
        counts = totals;
    }

    void evaluate(EvaluationEpoch const & when, Outcome result, Status code, bool attempted)
    {
        epoch = when;
        outcome = result;
        status = code;
        transaction_status = Status::success;
        gate_valid = innovation_limited = false;
        dimension = 0;
        innovation = variance = test_ratio = fusion_innovation = {};
        ++counts.evaluations;
        counts.attempts += attempted;
        counts.corrected += result == Outcome::corrected;
        counts.rejected += attempted && result == Outcome::rejected;
        counts.failed += attempted && (result == Outcome::failed || result == Outcome::rolled_back);
        if (result == Outcome::corrected)
        {
            last_correction_us = when.evaluation_time_us;
        }
    }

    template <typename Linalg, std::size_t Size>
    void record(EvaluationEpoch const & when, FusionResult<Linalg, Size> const & result, bool committed = true)
    {
        static_assert(Size <= 3);
        auto const action = result.decision == FusionDecision::fused
                                ? (committed ? Outcome::corrected : Outcome::rolled_back)
                            : result.decision == FusionDecision::rejected ? Outcome::rejected
                                                                          : Outcome::failed;
        evaluate(when, action, result.status, true);
        retain_gate(result);
    }

    template <typename Linalg, std::size_t Size>
    void record_check(EvaluationEpoch const & when, MeasurementCheck<Linalg, Size> const & result)
    {
        evaluate(when, succeeded(result.status) ? Outcome::checked : Outcome::failed, result.status, false);
        retain_gate(FusionResult<Linalg, Size>{result});
    }

    template <typename Linalg, std::size_t Size> void retain_gate(FusionResult<Linalg, Size> const & result)
    {
        static_assert(Size <= 3);
        gate_valid = result.diagnostics_valid;
        dimension = Size;
        if (gate_valid)
        {
            for (std::size_t axis = 0; axis < Size; ++axis)
            {
                innovation[axis] = fusion_innovation[axis] = result.innovation(axis);
                variance[axis] = result.innovation_variance(axis);
                test_ratio[axis] = result.test_ratio(axis);
            }
        }
    }

    void record_reset(EvaluationEpoch const & when, Status code, bool committed)
    {
        reset = {when, code, true, committed};
        counts.resets += committed;
        counts.reset_failures += !committed;
        if (committed)
        {
            last_reset_us = when.evaluation_time_us;
        }
    }
};

enum class InsOperation
{
    none,
    input,
    initialization,
    bias_reset,
    bias_learning,
    fusion_sequence,
    output_prediction,
    prediction,
    observation_selection,
    gnss_quality,
    gnss_control,
    gnss_velocity,
    gnss_position,
    vertical_health,
    height,
    gravity,
    magnetic,
    continuity,
    output_feedback,
    frame_change,
    output_preparation
};

} // namespace formal_eskf::runtime
