"""Selected application and framework source roots for repository tests."""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BINDU_ROOT = Path(os.environ.get("BINDU_TEST_SOURCE", ROOT/"artifacts/dependencies/bindu")).resolve()

os.environ.setdefault("BINDU_FRAMEWORK_SOURCE", str(BINDU_ROOT))
