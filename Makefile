.PHONY: pydoc

pydoc:
	mkdir -p ./pydoc
	cd nodes/controls/ && pydoc -w ./*.py
	mv nodes/controls/*.html pydoc/

clean:
	rm -rf ./pydoc/*
	find . -name "*.pyc" -type f -delete
	find . -name "*.html" -type f -delete
	find . -name "*.bag" -type f -delete