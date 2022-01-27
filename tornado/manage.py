from __future__ import annotations

import argparse
from pathlib import Path

from ferrite.manage.tree import make_components
from ferrite.manage.cli import add_parser_args, read_run_params, ReadRunParamsError, run_with_params

if __name__ == "__main__":
    source_dir = Path.cwd() / "ferrite/source"
    assert source_dir.exists()

    target_dir = Path.cwd() / "target"
    target_dir.mkdir(exist_ok=True)

    components = make_components(source_dir, target_dir)

    parser = argparse.ArgumentParser(
        description="Tornado power supply controller development automation tool",
        usage="python -m tornado.manage <component>.<task> [options...]",
    )
    add_parser_args(parser, components)

    args = parser.parse_args()

    try:
        params = read_run_params(args, components)
    except ReadRunParamsError as e:
        print(e)
        exit(1)

    run_with_params(params)
