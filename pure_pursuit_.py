"""Backward-compatible entrypoint for pure pursuit CLI."""

from cli import main


if __name__ == "__main__":
    raise SystemExit(main())
