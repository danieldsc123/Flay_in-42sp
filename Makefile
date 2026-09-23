PYTHON ?= python3
VENV ?= .venv
BIN = $(VENV)/bin
MAP ?= easy_01_linear_path.txt
ARGS ?=

.PHONY: install run debug clean lint lint-strict

install:
	$(PYTHON) -m venv $(VENV)
	$(BIN)/python -m pip install -r requirements.txt

run:
	$(PYTHON) main.py $(ARGS) "$(MAP)"

debug:
	$(PYTHON) -m pdb main.py $(ARGS) "$(MAP)"

clean:
	find . -type d \( -name __pycache__ -o -name .mypy_cache -o -name .pytest_cache \) -prune -exec rm -rf {} +
	find . -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete

lint:
	$(BIN)/flake8 .
	$(BIN)/mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs

lint-strict:
	$(BIN)/flake8 .
	$(BIN)/mypy . --strict
