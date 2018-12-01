#
# Makefile for all magic stuff inside the report
#  @author Rafael Lima
#

FIGDIR = ../tex/figs/

# Dot stuff
GVFILES = $(wildcard dot/*.gv)
PICFILES = $(addprefix $(FIGDIR),$(notdir $(GVFILES:%.gv=%.png)))

# Rule them all
all: $(PICFILES)

# Dot figures
$(FIGDIR)%.png: dot/%.gv
	dot -Tpng $^ -o $@

clear:
	rm -v $(PICFILES)
