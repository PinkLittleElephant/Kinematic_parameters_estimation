OUTPUT=build
MAIN_TEX_FILE_NAME=ICPS_manipulator
PDF_PATH=./build/$(MAIN_TEX_FILE_NAME).pdf
TEX = -shell-escape -interaction=nonstopmode -file-line-error

all: pdf

pdf:
	@pdflatex $(TEX) -output-directory=$(OUTPUT) $(MAIN_TEX_FILE_NAME).tex

show: view

view:
	@gio open $(PDF_PATH)
	@echo "Wow, looks nice!"

clean:
	@echo "I cleaning up your folder build!:D"
	@rm -rf ./build/*
