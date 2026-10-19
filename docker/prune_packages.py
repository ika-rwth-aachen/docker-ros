#!/usr/bin/env python3
"""Keep only upstream source packages required by the target repository.

Arguments are the target and upstream source directories. All packages found in
target are roots; colcon computes their recursive dependency closure across both
directories, including runtime and test dependencies. Upstream package directories
outside that closure are deleted before rosdep and builds. Target sources are
never modified. If target contains no packages, all upstream packages are kept
to support repositories consisting only of .repos files.

Discovery completes before deletion begins. Colcon logging is disabled because
industrial_ci can run this script from a read-only target checkout.
"""

import pathlib
import shutil
import subprocess
import sys


def list_packages(*args):
    # industrial_ci runs hooks from a read-only target repository.
    result = subprocess.run(
        ["colcon", "--log-base", "/dev/null", "list", *args],
        check=True, text=True, stdout=subprocess.PIPE,
    )
    return result.stdout.splitlines()


def main():
    target = pathlib.Path(sys.argv[1]).resolve()
    upstream = pathlib.Path(sys.argv[2]).resolve()
    targets = list_packages("--base-paths", str(target), "--names-only")
    if not targets:
        print("No target packages found; keeping all upstream packages", flush=True)
        return

    selected = set(list_packages(
        "--base-paths", str(target), str(upstream),
        "--packages-up-to", *targets, "--names-only",
    ))
    # Finish graph discovery successfully before modifying the workspace.
    packages = list_packages("--base-paths", str(upstream))
    for package in packages:
        name, path, _ = package.split("\t", 2)
        if name not in selected:
            directory = pathlib.Path(path).resolve()
            if directory == upstream or upstream not in directory.parents:
                raise RuntimeError("Refusing to remove package outside upstream: " + path)
            print("Removing unused upstream package: " + name, flush=True)
            shutil.rmtree(directory)


if __name__ == "__main__":
    main()
