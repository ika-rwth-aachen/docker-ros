#!/usr/bin/env python

import pathlib
import subprocess
import sys
from typing import List


def findDotRepos(search_path: str) -> List[pathlib.Path]:

    return pathlib.Path(search_path).glob("**/*.repos")


def main():

    search_path = sys.argv[1] if len(sys.argv) > 1 else "."
    clone_path = sys.argv[2] if len(sys.argv) > 2 else "."
    cloned_repos = []

    while True:

        found_repos = findDotRepos(search_path)
        remaining_repos = set(found_repos) - set(cloned_repos)

        if not remaining_repos:
            break

        next_repo = list(remaining_repos)[0]
        with open(str(next_repo), "r") as f:
            proc = subprocess.run(["vcs", "import", clone_path, "--recursive"], stdin=f)
            if proc.returncode != 0:
                raise RuntimeError("vcs import failed")
        
        cloned_repos.append(next_repo)


if __name__ == "__main__":
    main()
