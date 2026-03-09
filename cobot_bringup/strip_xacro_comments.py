import re
import subprocess
import sys


XML_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: python -m cobot_bringup.strip_xacro_comments <xacro-file> [xacro-args...]", file=sys.stderr)
        return 2

    cmd = ["xacro", *sys.argv[1:]]
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as exc:
        if exc.stdout:
            sys.stdout.write(exc.stdout)
        if exc.stderr:
            sys.stderr.write(exc.stderr)
        return exc.returncode

    cleaned = XML_COMMENT_RE.sub("", result.stdout)
    sys.stdout.write(cleaned)
    if cleaned and not cleaned.endswith("\n"):
        sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
