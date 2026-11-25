# INS Bias Reset Tests

**Status:** Draft

**Scope:** [E-BIAS-RESET](../../../requirements/core/bias-reset.md), direct
selected nominal bias and covariance reset. Source identity/order/history and
runtime recovery are outside this pure operation.

**Level / purpose:** Core unit/regression tests.

**Profiles:** INS binary32/binary64 on enabled backends. The reset performs only
assignments after validating consumed priors; no codegen or approximation branch.
Exact equality and coefficient-bit preservation apply; no numerical tolerance.

**Oracle:** A small integer factor defines correlated SPD P or rank-two PSD P.
An independently assembled complementary principal submatrix plus new diagonal
checks all coefficients, nominal fields and all four selection masks. Quaternion
setup uses exactly representable unit coefficients. The no-op/failed frame is
compared coefficient by coefficient, including signed zero; padding is ignored.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-E-BIAS-RESET-01 | Reset only selected bias blocks | [Reset algebra](../../../requirements/core/bias-reset.md#reset-algebra) | All four group selections; unequal nonzero biases, correlated P and per-axis priors. | Selected biases zero; selected P rows/columns zero except new diagonal; every complementary coefficient and nominal p/v/q unchanged, by independent block oracle. | Native: [test_bias_reset.cpp](../../../../../tests/core/test_bias_reset.cpp) |
| TC-E-BIAS-RESET-02 | Reject invalid consumed priors atomically | [Validation](../../../requirements/core/bias-reset.md#validation-and-publication) | Put NaN, either infinity and negative finite values on every selected axis; combine negative and non-finite values in opposite scan orders. | Non-finite precedence before negative-domain failure; complete state/P unchanged; no successful partial reset. | Native: [test_bias_reset.cpp](../../../../../tests/core/test_bias_reset.cpp) |
| TC-E-BIAS-RESET-03 | Ignore unused reset data | [Validation](../../../requirements/core/bias-reset.md#validation-and-publication) | Invalid priors only in an unselected group; neither selected with all unused fields invalid. | Selected reset succeeds without consuming unused fields; empty selection is exact no-op. Input state/P still satisfy caller premises. | Native: [test_bias_reset.cpp](../../../../../tests/core/test_bias_reset.cpp) |
| TC-E-BIAS-RESET-04 | Preserve singular and extreme valid covariance | [Reset algebra](../../../requirements/core/bias-reset.md#reset-algebra) | Singular correlated PSD P, zero and largest finite reset variances, signed-zero complementary coefficients; repeat reset. | Direct block result without overflow from unnecessary products; complementary bits preserved; second identical reset idempotent. Does not accept invalid input P. | Native: [test_bias_reset.cpp](../../../../../tests/core/test_bias_reset.cpp) |

## Review boundary

Native fixtures establish the listed executable examples and exact preservation
checks, not universal PSD/symmetry or IEEE/source proofs. The input state/P validity
premise remains with callers. Runtime event admission, prior conversion and output
feedback must be accepted independently before source-continuation support is claimed.
