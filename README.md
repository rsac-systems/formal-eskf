# formal-eskf

A portable C++20 inertial navigation system (INS) based on an error-state Kalman filter (ESKF).

## Key Features

- **Sensor fusion:** IMU prediction with GNSS, magnetometer and barometer aiding.
- **Portable design:** platform-independent core and runtime with Eigen, PX4 matrix and CMSIS-DSP backends.
- **VeriCoding-inspired development:** AI-assisted code and proof construction with explicit specifications and machine verification.
- **Traceability:** links requirements, code, formal proofs and test evidence.

## Quick Start

Requires a C++20 compiler, CMake 3.21 or newer, Eigen 3.3 or newer and Python 3
for the test tools. On Ubuntu / Debian:

```bash
sudo apt install git build-essential cmake libeigen3-dev python3
git clone https://github.com/recos-dev/formal-eskf.git
cd formal-eskf
cmake -S . -B build/demo -DBUILD_TESTING=ON -DFORMAL_ESKF_BUILD_EXAMPLES=ON
cmake --build build/demo --parallel 2
./build/demo/ins_runtime
```

The example uses synthetic stationary data; no dataset is needed. Run the tests:

```bash
python3 scripts/verify_tests.py --build-dir build/demo --no-build --ctest
```

For interactive configuration, use `make menuconfig` followed by `make test`.
See the [configuration guide](docs/guides/configuration.md#kconfiglib-integration) for options.

## Examples and Integration

Replay PX4 ULogs or KITTI Raw OXTS data through the INS runtime.
Install the replay dependencies in a Python environment that permits package installation:

```bash
sudo apt install python3-pip
python3 -m pip install -r requirements.txt
```

Datasets are not bundled. For PX4 module setup, see [SITL integration](integrations/px4/README.md).

### PX4 ULog

```bash
python3 examples/replay/px4.py /path/to/flight.ulg --plot
```

<details>
<summary>Download and replay a sample flight</summary>

From [PX4 Flight Review](https://review.px4.io/plot_app?log=e8744135-8493-4f5b-bab8-625c5b287d5a), with `curl` installed:

```bash
LOG_ID=e8744135-8493-4f5b-bab8-625c5b287d5a
curl --fail --location "https://review.px4.io/download?log=${LOG_ID}" --output "${LOG_ID}.ulg"
python3 examples/replay/px4.py "${LOG_ID}.ulg" --plot
```

</details>

### KITTI Raw

Use an unsynced (`_extract`) drive containing `oxts/`:

```bash
python3 examples/replay/kitti.py /path/to/2011_10_03_drive_0027_extract --plot
```

OXTS GPS/INS estimates initialize and correct the filter; they are not independent
ground truth. See the [KITTI assumptions](docs/guides/navigation-runtime.md#recorded-kitti-inputs).

<details>
<summary>Download and replay a sample drive</summary>

From [KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php), with `curl` and
`unzip` installed. The archive is approximately 27 GB; only OXTS is extracted.

```bash
DRIVE_ID=2011_10_03_drive_0027
curl --fail --location \
    "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/${DRIVE_ID}/${DRIVE_ID}_extract.zip" \
    --output "${DRIVE_ID}_extract.zip"
unzip -n "${DRIVE_ID}_extract.zip" "2011_10_03/${DRIVE_ID}_extract/oxts/*"
python3 examples/replay/kitti.py "2011_10_03/${DRIVE_ID}_extract" --plot
```

</details>

Results and plots are saved under `output/`, named after the log or drive.
Rerunning the same input replaces its previous output.

## Verification and Traceability

Lean 4 proves mathematical properties; ESBMC checks selected C++ behavior.
Tests, static analysis and sanitizers complement these checks.

The [traceability map](formal/agent-review/traceability-map.json) links
[requirements](docs/spec/README.md), code, proofs and test evidence for agent-assisted review.
Proof claims apply to specified properties and configurations; see the
[verification policy](docs/spec/verification/assurance.md) and [verification guide](docs/guides/verification.md).

This specification-driven approach and retained evidence support future ISO
certification work; the project is not certified.

## Documentation

| Topic | Entry point |
|---|---|
| Architecture and usage | [Architecture](docs/spec/architecture/overview.md), [runtime guide](docs/guides/navigation-runtime.md) |
| Configuration | [Build and deployment settings](docs/guides/configuration.md) |
| Specifications | [Documentation index](docs/README.md) |
| Planned work | [Roadmap](docs/roadmap.md) |
| Contributing | [Coding style](docs/guides/coding-style.md), [C++ subset](docs/spec/requirements/common/cpp-subset.md) |

## References

- [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508). 2017.
- [A Computationally Efficient GNSS/INS Design of Multirotor based on Error-state Kalman Filter](https://doi.org/10.23919/SICE59929.2023.10354209). 2023.
- [A benchmark for vericoding: formally verified program synthesis](https://arxiv.org/abs/2509.22908). 2025.

## License

This project is licensed under the [BSD 3-Clause License](LICENSE).
