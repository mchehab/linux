# -*- makefile -*-
# Makefile for Sphinx documentation
#

# for cleaning
subdir- := devicetree/bindings

ifneq ($(MAKECMDGOALS),cleandocs)
# Check for broken documentation file references
ifeq ($(CONFIG_WARN_MISSING_DOCUMENTS),y)
$(shell $(srctree)/scripts/documentation-file-ref-check --warn)
endif

# Check for broken ABI files
ifeq ($(CONFIG_WARN_ABI_ERRORS),y)
$(shell $(srctree)/scripts/get_abi.py --dir $(srctree)/Documentation/ABI validate)
endif
endif

# You can set these variables from the command line.
SPHINXBUILD   = sphinx-build
SPHINXOPTS    =
SPHINXDIRS    = .
DOCS_THEME    =
DOCS_CSS      =
_SPHINXDIRS   = $(sort $(patsubst $(srctree)/Documentation/%/index.rst,%,$(wildcard $(srctree)/Documentation/*/index.rst)))
SPHINX_CONF   = conf.py
PAPER         =
BUILDDIR      = $(obj)/output
PDFLATEX      = xelatex
LATEXOPTS     = -interaction=batchmode -no-shell-escape
BUILD_WRAPPER = $(srctree)/scripts/sphinx-build-wrapper

PYTHONPYCACHEPREFIX ?= $(abspath $(BUILDDIR)/__pycache__)

# For denylisting "variable font" files
# Can be overridden by setting as an env variable
FONTS_CONF_DENY_VF ?= $(HOME)/deny-vf


# User-friendly check for sphinx-build
HAVE_SPHINX := $(shell if which $(SPHINXBUILD) >/dev/null 2>&1; then echo 1; else echo 0; fi)

ifeq ($(HAVE_SPHINX),0)

.DEFAULT:
	$(warning The '$(SPHINXBUILD)' command was not found. Make sure you have Sphinx installed and in PATH, or set the SPHINXBUILD make variable to point to the full path of the '$(SPHINXBUILD)' executable.)
	@echo
	@$(srctree)/scripts/sphinx-pre-install
	@echo "  SKIP    Sphinx $@ target."

else # HAVE_SPHINX

# Common documentation targets
infodocs texinfodocs latexdocs epubdocs xmldocs pdfdocs linkcheckdocs:
	$(Q)@$(srctree)/scripts/sphinx-pre-install --version-check
	+$(Q)$(PYTHON3) $(BUILD_WRAPPER) $@ \
		--sphinxdirs="$(SPHINXDIRS)" \
		--conf=$(SPHINX_CONF) \
		--theme=$(DOCS_THEME) \
		--css=$(DOCS_CSS) \
		--paper=$(PAPER)

# Special handling for pdfdocs
ifeq ($(shell which $(PDFLATEX) >/dev/null 2>&1; echo $$?),0)
pdfdocs: DENY_VF = XDG_CONFIG_HOME=$(FONTS_CONF_DENY_VF)
else
pdfdocs:
	$(warning The '$(PDFLATEX)' command was not found. Make sure you have it installed and in PATH to produce PDF output.)
	@echo "  SKIP    Sphinx $@ target."
endif

infodocs: texinfodocs

# HTML main logic is identical to other targets. However, if rust is enabled,
# an extra step at the end is required to generate rustdoc.
htmldocs:
	$(Q)@$(srctree)/scripts/sphinx-pre-install --version-check
	+$(Q)$(PYTHON3) $(BUILD_WRAPPER) $@ \
		--sphinxdirs="$(SPHINXDIRS)" \
		--conf=$(SPHINX_CONF) \
		--theme=$(DOCS_THEME) \
		--css=$(DOCS_CSS) \
		--paper=$(PAPER)

# If Rust support is available and .config exists, add rustdoc generated contents.
# If there are any, the errors from this make rustdoc will be displayed but
# won't stop the execution of htmldocs

ifneq ($(wildcard $(srctree)/.config),)
ifeq ($(CONFIG_RUST),y)
	$(Q)$(MAKE) rustdoc || true
endif
endif

endif # HAVE_SPHINX

# The following targets are independent of HAVE_SPHINX, and the rules should
# work or silently pass without Sphinx.

refcheckdocs:
	$(Q)cd $(srctree);scripts/documentation-file-ref-check

cleandocs:
	$(Q)rm -rf $(BUILDDIR)
	$(Q)$(MAKE) BUILDDIR=$(abspath $(BUILDDIR)) $(build)=Documentation/userspace-api/media clean

dochelp:
	@echo  ' Linux kernel internal documentation in different formats from ReST:'
	@echo  '  htmldocs        - HTML'
	@echo  '  texinfodocs     - Texinfo'
	@echo  '  infodocs        - Info'
	@echo  '  latexdocs       - LaTeX'
	@echo  '  pdfdocs         - PDF'
	@echo  '  epubdocs        - EPUB'
	@echo  '  xmldocs         - XML'
	@echo  '  linkcheckdocs   - check for broken external links'
	@echo  '                    (will connect to external hosts)'
	@echo  '  refcheckdocs    - check for references to non-existing files under'
	@echo  '                    Documentation'
	@echo  '  cleandocs       - clean all generated files'
	@echo
	@echo  '  make SPHINXDIRS="s1 s2" [target] Generate only docs of folder s1, s2'
	@echo  '  valid values for SPHINXDIRS are: $(_SPHINXDIRS)'
	@echo
	@echo  '  make SPHINX_CONF={conf-file} [target] use *additional* sphinx-build'
	@echo  '  configuration. This is e.g. useful to build with nit-picking config.'
	@echo
	@echo  '  make DOCS_THEME={sphinx-theme} selects a different Sphinx theme.'
	@echo
	@echo  '  make DOCS_CSS={a .css file} adds a DOCS_CSS override file for html/epub output.'
	@echo
	@echo  '  make PAPER={a4|letter} Specifies the paper size used for LaTeX/PDF output.'
	@echo
	@echo  '  Default location for the generated documents is Documentation/output'
