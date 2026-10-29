"""Exercise template optimization with a real ARM object and consumer link."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/optimize_template.py"


@unittest.skipUnless(shutil.which("arm-none-eabi-gcc"), "ARM toolchain required")
class TemplateOptimizationTest(unittest.TestCase):
    def test_debug_is_removed_but_library_still_links(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "library.c").write_text("int answer(void) { return 42; }\n")
            (root / "main.c").write_text("extern int answer(void); int main(void) { return answer(); }\n")
            subprocess.run(["arm-none-eabi-gcc", "-g", "-c", str(root / "library.c"),
                            "-o", str(root / "library.o")], check=True)
            subprocess.run(["arm-none-eabi-ar", "rcs", str(root / "original.a"),
                            str(root / "library.o")], check=True)
            original = (root / "original.a").read_bytes()
            archive = root / "template.zip"
            with zipfile.ZipFile(archive, "w") as package:
                package.writestr("firmware/autotune.a", original)
                package.writestr("template.pros", '{"metadata": {}}')
            subprocess.run([sys.executable, str(SCRIPT), str(archive)], check=True)
            with zipfile.ZipFile(archive) as package:
                optimized = package.read("firmware/autotune.a")
                self.assertLess(len(optimized), len(original))
                self.assertEqual(package.read("template.pros"), b'{"metadata": {}}')
                self.assertEqual(package.getinfo("firmware/autotune.a").compress_type, zipfile.ZIP_DEFLATED)
            self.assertEqual((root / "original.a").read_bytes(), original)
            (root / "optimized.a").write_bytes(optimized)
            subprocess.run(["arm-none-eabi-gcc", "-nostdlib", "-Wl,-e,main",
                            str(root / "main.c"), str(root / "optimized.a"),
                            "-o", str(root / "consumer.elf")], check=True)
            sections = subprocess.check_output(["arm-none-eabi-readelf", "-S", str(root / "optimized.a")], text=True)
            self.assertNotIn(".debug_info", sections)


if __name__ == "__main__":
    unittest.main()
