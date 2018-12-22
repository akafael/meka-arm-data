#
# Makefile for all magic stuff inside the report
#  @author Rafael Lima
#

# Directories ------------------------------------------------------------------------------
FIGDIR = ../tex/figs
LOGDIR = logs

# Target Files -----------------------------------------------------------------------------
# Dot stuff
GVFILES = $(wildcard dot/*.gv)
PICFILES = $(addprefix $(FIGDIR)/,$(notdir $(GVFILES:%.gv=%.png)))

# Experiment log files
BAGFILES = $(wildcard $(LOGDIR)/*.bag)
CSVFILES = $(BAGFILES:%.bag=%.csv) $(addpreffix $(LOGDIR)/control_,$(notdir $(BAGFILES:%.bag=control_%.csv)))
BAGREADER = python/bag_reader.py


# Common Rules -----------------------------------------------------------------------------
# One rule to rule them all
all: $(PICFILES) $(CSVFILES)

# Remove Generated Files
clear:
	rm -v $(PICFILES) $(CSVFILES)

# Images -----------------------------------------------------------------------------------
# Dot figures
$(FIGDIR)/%.png: dot/%.gv
	dot -Tpng $^ -o $@

# ROS Bag ----------------------------------------------------------------------------------
# Humanoid State topic
$(LOGDIR)/%.csv: $(LOGDIR)/%.bag
	python $(BAGREADER) -b $< -t /humanoid_state -o $@
	sed -i -e '1s/name/name_j0,name_j1,name_j2,name_j3,name_j4,name_j5,name_j6,name_gr/' $@
	sed -i -e '1s/effort/effort_j0,effort_j1,effort_j2,effort_j3,effort_j4,effort_j5,effort_j6,effort_gr/' $@
	sed -i -e '1s/position/position_j0,position_j1,position_j2,position_j3,position_j4,position_j5,position_j6,position_gr/' $@
	sed -i -e '1s/velocity/velocity_j0,velocity_j1,velocity_j2,velocity_j3,velocity_j4,velocity_j5,velocity_j6,velocity_gr/' $@
	sed -i -e 's/``/,/g' $@

# Humanoid Command topic
$(LOGDIR)/control_%.csv: $(LOGDIR)/%.bag
	python $(BAG_READER) -b $< -t /humanoid_command -o $@
	sed -i -e '1s/chain_idx/chain_idx_j0,chain_idx_j1,chain_idx_j2,chain_idx_j3,chain_idx_j4,chain_idx_j5,chain_idx_j6/' $@
	sed -i -e '1s/effort/effort_j0,effort_j1,effort_j2,effort_j3,effort_j4,effort_j5,effort_j6/' $@
	sed -i -e '1s/stiffness/stiffness_j0,stiffness_j1,stiffness_j2,stiffness_j3,stiffness_j4,stiffness_j5,stiffness_j6/' $@
	sed -i -e '1s/position/position_j0,position_j1,position_j2,position_j3,position_j4,position_j5,position_j6/' $@
	sed -i -e '1s/velocity/velocity_j0,velocity_j1,velocity_j2,velocity_j3,velocity_j4,velocity_j5,velocity_j6/' $@
	sed -i -e 's/``/,/g' $@
