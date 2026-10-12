from bucky.firmware.build import firmware_available
from bucky.lab.experiments import drive_approach  # noqa: F401  (registers the experiment)
from bucky.lab.experiments.base import (
    Experiment,
    get_experiment,
    list_experiments,
    register_experiment,
    summarize_metrics,
)

if firmware_available():   # experiments for the real firmware (robot/src), when it is present
    from bucky.firmware import ekf_bench, sensor_check  # noqa: E402,F401

__all__ = [
    "Experiment", "get_experiment", "list_experiments", "register_experiment", "summarize_metrics",
]
