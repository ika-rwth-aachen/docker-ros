#!/usr/bin/env python

import pathlib
import subprocess
import sys
from typing import List, Optional


def findDotRepos(search_path: str, clone_path: Optional[str] = None, repos_file: str = ".repos") -> List[pathlib.Path]:

    repos = list(pathlib.Path(search_path).glob("**/*.repos"))
    if clone_path is not None:
        repos.extend(list(pathlib.Path(clone_path).glob("**/*.repos"))) # find .repos in upstream dependencies
    else:
        repos = list(pathlib.Path(search_path).glob(f"**/*{repos_file}"))      # find .repos in target repo
    return repos

def main():

    search_path = sys.argv[1] if len(sys.argv) > 1 else "."
    clone_path = sys.argv[2] if len(sys.argv) > 2 else "."
    repos_file = sys.argv[3] if len(sys.argv) > 3 else ".repos"
    cloned_repos = []

    while True:

        found_repos = findDotRepos(search_path, clone_path, repos_file)
        remaining_repos = set(found_repos) - set(cloned_repos)

        if not remaining_repos:
            break

        next_repo = list(remaining_repos)[0]
        with open(str(next_repo), "r") as f:
            proc = subprocess.run(["vcs", "import", clone_path, "--recursive"], stdin=f)
            if proc.returncode != 0:
                raise RuntimeError("vcs import failed")

        cloned_repos.append(next_repo)

    print(" ".join([str(repo) for repo in set(found_repos)]))


if __name__ == "__main__":
    main()