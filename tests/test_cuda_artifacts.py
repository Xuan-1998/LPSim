import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.check_cuda_artifacts import (  # noqa: E402
    inspect_binary,
    missing_architectures,
    parse_architectures,
)


class CudaArtifactTests(unittest.TestCase):
    def test_parse_architectures_from_cubin_listing(self):
        output = """
        ELF file    1: lpsim.1.sm_80.cubin
        ELF file    2: lpsim.2.sm_89.cubin
        ELF file    3: lpsim.3.sm_100.cubin
        """

        self.assertEqual(parse_architectures(output), {80, 89, 100})

    def test_parse_architectures_accepts_compute_targets_and_suffixes(self):
        output = "arch = sm_90\n.target compute_100a\n"

        self.assertEqual(parse_architectures(output), {90, 100})

    @patch("tools.check_cuda_artifacts.subprocess.run")
    def test_inspect_binary_queries_native_and_ptx_images(self, run):
        run.side_effect = [
            subprocess.CompletedProcess([], 0, "lpsim.sm_90.cubin\n", ""),
            subprocess.CompletedProcess(
                [], 0, "lpsim.sm_80.ptx\nlpsim.sm_100.ptx\n", ""
            ),
        ]

        native, ptx = inspect_binary(Path("build/lpsim"), "custom-cuobjdump")

        self.assertEqual(native, {90})
        self.assertEqual(ptx, {80, 100})
        self.assertEqual(
            run.call_args_list[0].args[0],
            ["custom-cuobjdump", "--list-elf", "build/lpsim"],
        )
        self.assertEqual(
            run.call_args_list[1].args[0],
            ["custom-cuobjdump", "--list-ptx", "build/lpsim"],
        )

    def test_missing_architectures_reports_only_absent_targets(self):
        self.assertEqual(
            missing_architectures({80, 90}, {80, 89, 90, 100}), {89, 100}
        )


if __name__ == "__main__":
    unittest.main()
