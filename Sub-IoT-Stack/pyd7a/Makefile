COVERAGE=PYTHONPATH=. $(shell which coverage3)

all: clean test coverage

test:
	@echo "*** running all tests"
	@PYTHONPATH="../" $(COVERAGE) run -m pytest test/ --junitxml=unittest-result.xml

test-%:
	@echo "*** performing tests for d7a-$(subst _,/,$(subst test-,,$@))"
	@PYTHONPATH="../" $(COVERAGE) run -m pytest test/d7a/$(subst -,/,$(subst test-,,$@))/*.py --junitxml=unittest-result.xml

coverage:
	@echo "*** generating unittest coverage report (based on last test run)"
	@$(COVERAGE) report -m --omit '/System/*,*__init__.py,test/*,*site-packages*,*/support/*,/usr/*'

clean:
	@rm -f .coverage

.PHONY: all test clean test test-% coverage
