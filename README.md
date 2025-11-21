# Python Exosuit
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/template-python/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/exosuit-python?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/exosuit-python/actions/workflows/ci.yml/badge.svg)

This repo is the main codebase to run the lower-limb exosuit on a single board computer like the Jetson Nano.

## Install
To install the library run:

```bash
uv install python-exosuit
```

OR

```bash
uv install git+https://github.com/TUM-Aries-Lab/python-exosuit.git@<specific-tag>
```

## Development
0. Install [uv](https://docs.astral.sh/uv/getting-started/installation/)
1. Install [pyenv](https://github.com/pyenv/pyenv?tab=readme-ov-file#installation)
2. ```pyenv install <desired-python-version>  # install the required python version```
3. ```pyenv global <desired-python-version>  # set the required python version```
4. ```git clone git@github.com:TUM-Aries-Lab/exosuit-python.git```
5. `make init` to create the virtual environment and install dependencies
6. `make format` to format the code and check for errors
7. `make test` to run the test suite
8. `make clean` to delete the temporary files and directories

## Publishing
It's super easy to publish your own packages on PyPI. To build and publish this package run:

```bash
uv build
uv publish  # make sure your version in pyproject.toml is updated
```
The package can then be found at: https://pypi.org/project/exosuit-python

## Module Usage
```python
"""Basic docstring for my module."""

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
