# Python Exosuit
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/exosuit-python/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/exosuit-python?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/exosuit-python/actions/workflows/ci.yml/badge.svg)

This repo is the main codebase to run the lower-limb exosuit on a single board computer like the Jetson Nano.

## Install
To install the library run:

```bash
uv pip install exosuit-python==latest
```
OR
```bash
uv add git+https://github.com/TUM-Aries-Lab/exosuit-python.git@<specific-tag>  # need credentials
```

## Development
0. Install [uv](https://docs.astral.sh/uv/getting-started/installation/) from Astral.
1. `git clone git@github.com:TUM-Aries-Lab/exosuit-python.git`
2. Install the dependencies to use Makefiles.
3. `make init` to create the virtual environment and install dependencies
3. `make format` to format the code and check for errors
4. `make test` to run the test suite
5. `make clean` to delete the temporary files and directories

## Publishing
It's super easy to publish your own packages on PyPI. To build and publish this package run:

```bash
uv build
uv publish  # make sure your version in pyproject.toml is updated
```
The package can then be found at: https://pypi.org/project/exosuit-python

## Module Usage
```python
"""Basic docstring for the exosuit module."""

def main() -> None:
    """Run a simple demonstration."""
    logger.info("Hello World!")

if __name__ == "__main__":
    main()
```

## Program Usage
```bash
uv run python -m exosuit_python
```

## Structure
<!-- TREE-START -->
```
├── src
│   └── exosuit_python
│       ├── __init__.py
│       ├── __main__.py
│       ├── definitions.py
│       ├── exosuit.py
│       └── utils.py
├── tests
│   ├── __init__.py
│   ├── conftest.py
│   ├── main_test.py
│   └── utils_test.py
├── .dockerignore
├── .gitignore
├── .pre-commit-config.yaml
├── .python-version
├── CONTRIBUTING.md
├── Dockerfile
├── LICENSE
├── Makefile
├── README.md
├── pyproject.toml
├── repo_tree.py
└── uv.lock
```
<!-- TREE-END -->
