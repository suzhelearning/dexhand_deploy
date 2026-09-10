"""Guard the source tree against host-specific checkout paths."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = frozenset({
    ".cpp", ".h", ".hpp", ".json", ".md", ".py", ".sh", ".toml",
    ".txt", ".yaml", ".yml",
})
HOST_PATH = re.compile(rb"(?:^|[\s\"'`=:(])/(?:home|root|opt|mnt|workspace)/", re.MULTILINE)


class PortablePathHygieneTest(unittest.TestCase):
    def test_product_sources_and_docs_have_no_host_checkout_paths(self) -> None:
        roots = (
            ROOT / "src",
            ROOT / "scripts",
            ROOT / "docs",
            ROOT / "README.md",
            ROOT / "VENDOR_RUNTIME.md",
            ROOT / "third_party",
            ROOT / "tools",
        )
        violations: list[str] = []
        for root in roots:
            paths = (root,) if root.is_file() else root.rglob("*")
            for path in paths:
                if {".pixi", "build", "runtime"}.intersection(path.parts):
                    continue
                if not path.is_file() or path.suffix not in TEXT_SUFFIXES:
                    continue
                if HOST_PATH.search(path.read_bytes()):
                    violations.append(path.relative_to(ROOT).as_posix())
        self.assertEqual([], violations)


if __name__ == "__main__":
    unittest.main()
