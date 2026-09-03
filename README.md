[![GPLv3 License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This is a **Python translation for learning** Apollo's lattice planner (not a production simulator). After refactoring, all modules should import cleanly for reading and white-box study.

### Setup

Create and activate your own virtual environment (venv, conda, etc. — none is bundled with this repo), then:

```bash
pip install -r requirements.txt
python scripts/verify_imports.py
```

Many methods remain `NotImplementedError` stubs mirroring the original C++ port scope; the lattice **planning pipeline** in `LatticePlanner.py` is the main readable path.

## Code Translation Purpose
Since Baidu Apollo's lattice planner is embedded in the planning module using C++, it has a very high degree of coupling. 
This has made it difficult to conduct effective white-box validation and verification on the lattice algorithm. We have extracted the core
processing algorithm and related support classes of this planner and translated them into Python code to assist people to get a better understand of this component.

## Notice
Notably, the goal of this project is not to build a functional planner in a distinct simulator to do experiments, but to learn the algorithms and data structures within it. Therefore, we are translating the C++ functions in the Lattice algorithm into their corresponding Python frameworks as much as possible. Due to the introduction of CyberRT, the analysis of the program has also caused confusion. We are consolidating the protobuf message objects used by this planner into Python dataclasses to clearly demonstrate how the planner manipulates data.