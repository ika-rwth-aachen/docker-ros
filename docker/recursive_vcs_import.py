#!/usr/bin/env python

import pathlib
import subprocess
import sys
from typing import List, Optional


def find_dot_repos(search_path: str, clone_path: Optional[str] = None) -> List[pathlib.Path]:

    repos = list(pathlib.Path(search_path).glob("**/*.repos"))
    if clone_path is not None:
        repos.extend(list(pathlib.Path(clone_path).glob("**/*.repos")))
    return repos

def is_file_empty(filename):
    """Check if file is empty, allowing blank spaces."""
    try:
        with open(filename, 'r') as file:
            content = file.read().strip()
            return len(content) == 0
    except Exception as e:
        print(e)
        # Exception gets raised later
        return False

def main():

    search_path = sys.argv[1] if len(sys.argv) > 1 else "."
    clone_path = sys.argv[2] if len(sys.argv) > 2 else "."
    cloned_repos = []

    while True:

        found_repos = find_dot_repos(search_path, clone_path)
        remaining_repos = set(found_repos) - set(cloned_repos)

        if not remaining_repos:
            break

        next_repo = list(remaining_repos)[0]
        with open(str(next_repo), "r") as f:
            proc = subprocess.run(["vcs", "import", clone_path, "--recursive"], stdin=f)
            if proc.returncode != 0:
                # Ignore empty .repos files
                if not is_file_empty(next_repo):
                    raise RuntimeError("vcs import failed")
                else:
                    print(f"Ignored empty .repos file: {next_repo}")

        cloned_repos.append(next_repo)

    print(" ".join([str(repo) for repo in set(found_repos)]))


if __name__ == "__main__":
    main()
    