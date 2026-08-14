"""Module entrypoint for `python -m g1_teach_v2`.

保持一个很薄的启动层，这样 CLI 逻辑仍然集中在 `cli.py` 里维护。
"""

from .cli import main


if __name__ == "__main__":
    raise SystemExit(main())
