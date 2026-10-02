PYTHON ?= .venv/bin/python
.PHONY: all setup test quick
all:
	$(PYTHON) run_pipeline.py
setup:
	uv venv --python 3.12 .venv
	uv pip install --python $(PYTHON) -r requirements.txt
test:
	$(PYTHON) -m pytest tests -q
quick:
	$(PYTHON) run_pipeline.py --quick
