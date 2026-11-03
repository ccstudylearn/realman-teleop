# Third-party components

The repository preserves the original project's MIT notice in `LICENSE`.

| Component | Version / source | Distribution / notice |
| --- | --- | --- |
| XRoboToolkit Quest client | Upstream commit `07a28333341629008187e988b73b48bced316092`, patched version `1.0.1-realman.2`; https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest | Modified APK bundled in release archives or downloaded to `vendor/quest-realman.apk`; MIT notice in `licenses/XRoboToolkit_Quest_MIT.txt`; patches and build script in `client/` |
| XRoboToolkit PC Service / native SDK | v1.0.0, Ubuntu 24.04 amd64; https://github.com/XR-Robotics/XRoboToolkit-PC-Service | Downloaded/extracted unchanged by setup; complete release also carries original DEB; Apache-2.0 text in `licenses/XRoboToolkit_PC_Service_Apache-2.0.txt`; retain upstream notices in the DEB |
| RealMan Python / offline algorithm SDK | `Robotic_Arm==1.1.4`; https://pypi.org/project/Robotic-Arm/1.1.4/ | Installed from original wheel; complete release carries the unchanged wheel with its vendor files and notices |
| NumPy | 2.2.6; https://numpy.org/ | Original wheel; its bundled licenses and notices remain inside the wheel |
| SciPy | 1.15.3; https://scipy.org/ | Original wheel; its bundled licenses and notices remain inside the wheel |

The Quest APK includes dependencies packaged by the upstream Unity/Meta project. This repository does not relicense those components. Rebuilding requires the upstream project's packages and their terms. Unity editor, account credentials, editor activation data and private signing keys are not included.

The RealMan implementation follows the input/IK/backend separation referenced from https://github.com/GZF123/arx-televr at commit `cd557afbb9ec280df2d9e12e1582219b85ee1a53`; no ARX robot model, native executable, CAN backend or Home behavior is shipped.

Dual-camera capture additionally uses unchanged wheels of opencv-python-headless 4.12.0.88 (https://pypi.org/project/opencv-python-headless/4.12.0.88/) and pyrealsense2 2.58.4.10922 (https://pypi.org/project/pyrealsense2/2.58.4.10922/); upstream licenses/notices remain inside each original wheel.
