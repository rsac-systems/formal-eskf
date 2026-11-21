/-
formal-eskf is freely redistributable under the BSD 3-Clause License.
See the file "LICENSE" for information on usage and redistribution of this file.
SPDX-License-Identifier: BSD-3-Clause
-/
import FormalESKF.MagneticFrame
import FormalESKF.Covariance
import Mathlib.Analysis.SpecialFunctions.Trigonometric.Arctan

/-! Exact-real WGS84 forward geometry and fixed-origin NED (F-GEO-ECEF/F-GEO-NED).
Angles below are radians. Reuse the existing geographic basis, rather than
introducing a second convention. These results do not certify libm, rounding,
the eight-step inverse's convergence, or a moving estimator origin. -/
namespace FormalESKF.Geodesy
noncomputable section
open Matrix SO3 ESKF

def semiMajor : ℝ := 6378137
def flattening : ℝ := 1 / (298257223563 / 1000000000)
def eccentricitySquared : ℝ := flattening * (2 - flattening)
def semiMinor : ℝ := semiMajor * (1 - flattening)

theorem ellipsoid_constants : 0 < semiMajor ∧ 0 < flattening ∧ flattening < 1 ∧
    0 < eccentricitySquared ∧ eccentricitySquared < 1 := by
  norm_num [semiMajor, flattening, eccentricitySquared]

theorem semiMinor_squared : semiMinor ^ 2 = semiMajor ^ 2 * (1 - eccentricitySquared) := by
  unfold semiMinor eccentricitySquared
  ring

def normalDenominator (latitude : ℝ) : ℝ :=
  1 - eccentricitySquared * Real.sin latitude ^ 2

theorem normalDenominator_pos (latitude : ℝ) : 0 < normalDenominator latitude := by
  have h := Real.sin_sq_add_cos_sq latitude
  have hc := sq_nonneg (Real.cos latitude)
  have he := ellipsoid_constants
  have hs : Real.sin latitude ^ 2 ≤ 1 := by nlinarith
  have := mul_le_mul_of_nonneg_left hs (le_of_lt he.2.2.2.1)
  dsimp [normalDenominator]
  nlinarith [he.2.2.2.2]

def primeVertical (latitude : ℝ) : ℝ := semiMajor / Real.sqrt (normalDenominator latitude)

theorem primeVertical_pos (latitude : ℝ) : 0 < primeVertical latitude :=
  div_pos ellipsoid_constants.1 (Real.sqrt_pos.2 (normalDenominator_pos latitude))

theorem primeVertical_ge_semiMajor (latitude : ℝ) : semiMajor ≤ primeVertical latitude := by
  have hd : normalDenominator latitude ≤ 1 := by
    have := mul_nonneg (le_of_lt ellipsoid_constants.2.2.2.1) (sq_nonneg (Real.sin latitude))
    dsimp [normalDenominator]
    linarith
  have hs : Real.sqrt (normalDenominator latitude) ≤ 1 := by
    simpa using Real.sqrt_le_sqrt hd
  rw [primeVertical, le_div_iff₀ (Real.sqrt_pos.2 (normalDenominator_pos latitude))]
  nlinarith [ellipsoid_constants.1]

theorem primeVertical_normal (latitude : ℝ) :
    primeVertical latitude * normalDenominator latitude = semiMajor * Real.sqrt (normalDenominator latitude) := by
  have hs := Real.sq_sqrt (le_of_lt (normalDenominator_pos latitude))
  have hn := ne_of_gt (Real.sqrt_pos.2 (normalDenominator_pos latitude))
  unfold primeVertical
  field_simp
  rw [hs]

theorem primeVertical_squared (latitude : ℝ) :
    primeVertical latitude ^ 2 * normalDenominator latitude = semiMajor ^ 2 := by
  have hs := Real.sq_sqrt (le_of_lt (normalDenominator_pos latitude))
  have hn := ne_of_gt (Real.sqrt_pos.2 (normalDenominator_pos latitude))
  unfold primeVertical
  field_simp
  rw [hs]

def geodeticToEcef (latitude longitude height : ℝ) : Vector3 ℝ :=
  ![(primeVertical latitude + height) * Real.cos latitude * Real.cos longitude,
    (primeVertical latitude + height) * Real.cos latitude * Real.sin longitude,
    (primeVertical latitude * (1 - eccentricitySquared) + height) * Real.sin latitude]

def ellipsoidNormal (latitude longitude : ℝ) : Vector3 ℝ :=
  ![Real.cos latitude * Real.cos longitude, Real.cos latitude * Real.sin longitude, Real.sin latitude]

/-- Zero-height coordinates lie on the WGS84 ellipsoid, independent of longitude. -/
theorem surface_ellipsoid (latitude longitude : ℝ) :
    (geodeticToEcef latitude longitude 0 0) ^ 2 +
      (geodeticToEcef latitude longitude 0 1) ^ 2 +
      (geodeticToEcef latitude longitude 0 2) ^ 2 / (1 - eccentricitySquared) = semiMajor ^ 2 := by
  have he : 1 - eccentricitySquared ≠ 0 := ne_of_gt (sub_pos.mpr ellipsoid_constants.2.2.2.2)
  have hl : Real.cos longitude ^ 2 = 1 - Real.sin longitude ^ 2 := by
    nlinarith [Real.sin_sq_add_cos_sq longitude]
  have hp : Real.cos latitude ^ 2 = 1 - Real.sin latitude ^ 2 := by
    nlinarith [Real.sin_sq_add_cos_sq latitude]
  rw [← primeVertical_squared latitude]
  simp [geodeticToEcef, normalDenominator]
  field_simp
  ring_nf
  simp only [hl, hp]
  ring

theorem equator (longitude height : ℝ) :
    geodeticToEcef 0 longitude height =
      ![(semiMajor + height) * Real.cos longitude, (semiMajor + height) * Real.sin longitude, 0] := by
  simp [geodeticToEcef, primeVertical, normalDenominator]

theorem polar_normal : semiMajor * Real.sqrt (1 - eccentricitySquared) = semiMinor := by
  have hf : 0 ≤ 1 - flattening := sub_nonneg.mpr (le_of_lt ellipsoid_constants.2.2.1)
  have he : 1 - eccentricitySquared = (1 - flattening) ^ 2 := by
    unfold eccentricitySquared
    ring
  rw [he, Real.sqrt_sq hf]
  rfl

theorem north_pole (longitude height : ℝ) :
    geodeticToEcef (Real.pi / 2) longitude height = ![0, 0, semiMinor + height] := by
  have hn := primeVertical_normal (Real.pi / 2)
  simp [normalDenominator] at hn
  simp [geodeticToEcef, hn, polar_normal]

theorem south_pole (longitude height : ℝ) :
    geodeticToEcef (-(Real.pi / 2)) longitude height = ![0, 0, -(semiMinor + height)] := by
  have hn := primeVertical_normal (-(Real.pi / 2))
  simp [normalDenominator] at hn
  simp [geodeticToEcef, hn, polar_normal]

theorem normal_unit (latitude longitude : ℝ) :
    dotProduct (ellipsoidNormal latitude longitude) (ellipsoidNormal latitude longitude) = 1 := by
  simp [ellipsoidNormal, dotProduct, Fin.sum_univ_succ]
  nlinarith [Real.sin_sq_add_cos_sq latitude, Real.sin_sq_add_cos_sq longitude,
    mul_nonneg (sq_nonneg (Real.cos latitude)) (sq_nonneg (Real.cos longitude))]

/-- Ellipsoidal height is displacement along the geodetic normal. -/
theorem height_displacement (latitude longitude height : ℝ) :
    geodeticToEcef latitude longitude height =
      geodeticToEcef latitude longitude 0 + height • ellipsoidNormal latitude longitude := by
  ext i
  fin_cases i <;> simp [geodeticToEcef, ellipsoidNormal] <;> ring

/-- The inverse's vertical correction is the forward radial factor times sin(phi). -/
theorem inverse_vertical_identity (latitude longitude height : ℝ) :
    geodeticToEcef latitude longitude height 2 +
      eccentricitySquared * primeVertical latitude * Real.sin latitude =
      (primeVertical latitude + height) * Real.sin latitude := by
  simp [geodeticToEcef]
  ring

def horizontalRadius (point : Vector3 ℝ) : ℝ := Real.sqrt (point 0 ^ 2 + point 1 ^ 2)

theorem horizontal_forward (latitude longitude height : ℝ) (hc : 0 ≤ Real.cos latitude)
    (hh : -12000 ≤ height) :
    horizontalRadius (geodeticToEcef latitude longitude height) =
      (primeVertical latitude + height) * Real.cos latitude := by
  have hn : 0 ≤ primeVertical latitude + height := by
    have := primeVertical_ge_semiMajor latitude
    norm_num [semiMajor] at this
    linarith
  have hl := Real.sin_sq_add_cos_sq longitude
  have he : (geodeticToEcef latitude longitude height 0) ^ 2 +
      (geodeticToEcef latitude longitude height 1) ^ 2 =
      ((primeVertical latitude + height) * Real.cos latitude) ^ 2 := by
    simp [geodeticToEcef]
    calc
      _ = ((primeVertical latitude + height) * Real.cos latitude) ^ 2 *
          (Real.sin longitude ^ 2 + Real.cos longitude ^ 2) := by ring
      _ = _ := by rw [hl, mul_one]
  rw [horizontalRadius, he, Real.sqrt_sq (mul_nonneg hn hc)]

/-- For strictly positive horizontal radius, atan2(y,p) equals arctan(y/p).
This exact model is only for the nonpolar branch of the C++ inverse. -/
def inverseLatitudeStep (horizontal vertical latitude : ℝ) : ℝ :=
  Real.arctan ((vertical + eccentricitySquared * primeVertical latitude * Real.sin latitude) / horizontal)

theorem inverse_latitude_fixedPoint (latitude longitude height : ℝ)
    (hlo : -(Real.pi / 2) < latitude) (hhi : latitude < Real.pi / 2) (hh : -12000 ≤ height) :
    inverseLatitudeStep (horizontalRadius (geodeticToEcef latitude longitude height))
      (geodeticToEcef latitude longitude height 2) latitude = latitude := by
  have hc := Real.cos_pos_of_mem_Ioo ⟨hlo, hhi⟩
  have hn : primeVertical latitude + height ≠ 0 := by
    have := primeVertical_ge_semiMajor latitude
    norm_num [semiMajor] at this
    linarith
  rw [inverseLatitudeStep, inverse_vertical_identity, horizontal_forward _ _ _ hc.le hh,
    mul_div_mul_left _ _ hn, ← Real.tan_eq_sin_div_cos, Real.arctan_tan hlo hhi]

/-- At the true latitude, the inverse's normal projection recovers height. -/
theorem inverse_height_identity (latitude longitude height : ℝ) :
    (primeVertical latitude + height) * Real.cos latitude * Real.cos latitude +
      geodeticToEcef latitude longitude height 2 * Real.sin latitude -
      semiMajor * Real.sqrt (normalDenominator latitude) = height := by
  rw [← primeVertical_normal]
  change (primeVertical latitude + height) * Real.cos latitude * Real.cos latitude +
    (primeVertical latitude * (1 - eccentricitySquared) + height) * Real.sin latitude *
      Real.sin latitude - primeVertical latitude * (1 - eccentricitySquared * Real.sin latitude ^ 2) = height
  calc
    _ = (primeVertical latitude + height) * (Real.sin latitude ^ 2 + Real.cos latitude ^ 2) -
        primeVertical latitude := by ring
    _ = height := by rw [Real.sin_sq_add_cos_sq]; ring

def toNed (latitude longitude : ℝ) (origin point : Vector3 ℝ) : Vector3 ℝ :=
  geographicNedBasis latitude longitude *ᵥ (point - origin)

def fromNed (latitude longitude : ℝ) (origin point : Vector3 ℝ) : Vector3 ℝ :=
  origin + (geographicNedBasis latitude longitude)ᵀ *ᵥ point

theorem origin_zero (latitude longitude : ℝ) (origin : Vector3 ℝ) :
    toNed latitude longitude origin origin = 0 := by simp [toNed]

theorem from_to_ned (latitude longitude : ℝ) (origin point : Vector3 ℝ) :
    fromNed latitude longitude origin (toNed latitude longitude origin point) = point := by
  simp only [fromNed, toNed, mulVec_mulVec, geographicNedBasis_transpose_mul, one_mulVec]
  abel

theorem to_from_ned (latitude longitude : ℝ) (origin point : Vector3 ℝ) :
    toNed latitude longitude origin (fromNed latitude longitude origin point) = point := by
  simp [fromNed, toNed, mulVec_mulVec, geographicNedBasis_mul_transpose]

theorem basis_normSquared (latitude longitude : ℝ) (vector : Vector3 ℝ) :
    dotProduct (geographicNedBasis latitude longitude *ᵥ vector)
      (geographicNedBasis latitude longitude *ᵥ vector) = dotProduct vector vector := by
  rw [dotProduct_mulVec, vecMul_mulVec, geographicNedBasis_transpose_mul, vecMul_one]

/-- Down is opposite the ellipsoid normal, not generally opposite ECEF z. -/
theorem normal_is_up (latitude longitude : ℝ) :
    geographicNedBasis latitude longitude *ᵥ ellipsoidNormal latitude longitude = ![0, 0, -1] := by
  have ha : Real.cos latitude ^ 2 = 1 - Real.sin latitude ^ 2 := by
    nlinarith [Real.sin_sq_add_cos_sq latitude]
  have hl : Real.cos longitude ^ 2 = 1 - Real.sin longitude ^ 2 := by
    nlinarith [Real.sin_sq_add_cos_sq longitude]
  ext i
  fin_cases i <;> simp [geographicNedBasis, ellipsoidNormal, mulVec, dotProduct, Fin.sum_univ_succ] <;>
    ring_nf <;> simp only [ha, hl] <;> ring

theorem covariance_psd (latitude longitude : ℝ) {P : Matrix3 ℝ} (hP : P.PosSemidef) :
    (geographicNedBasis latitude longitude * P * (geographicNedBasis latitude longitude)ᵀ).PosSemidef :=
  sandwich_posSemidef _ hP

theorem frame_covariance_psd (a b c d : ℝ) {P : Matrix3 ℝ} (hP : P.PosSemidef) :
    (magneticFrameChange a b c d * P * (magneticFrameChange a b c d)ᵀ).PosSemidef :=
  sandwich_posSemidef _ hP

end
end FormalESKF.Geodesy
