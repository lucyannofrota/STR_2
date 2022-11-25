all: aux_objs problems complete

FLAGS = -lpthread -lrt -lm -D_REENTRANT

DBGCFLAGS = -O2 -Wall -Wextra #-DDEBUG -g3 -O0

AUX_OBJS_NAMES = aux_libs/str_functions #linked_list

PROBLEMS_NAMES = main p1 p2

define generate_object
	@echo "\n|Generating $(2).o"
	gcc $(DBGCFLAGS) -c $(1)$(2).c -o $(1)$(2).o
	@echo "|$(2).o successfully generated\n" 

endef

define generate_exec
	@echo "\n|Generating exec $(2)"

	$(call generate_object,,$(2))

	gcc $(DBGCFLAGS) $(2).c $(1)$(addsuffix .o,$(AUX_OBJS_NAMES)) -o $(2) ${FLAGS}
	@echo "\n|exec $(2) successfully generated\n"

endef


aux_objs:
	$(foreach OBJ_NAME,$(AUX_OBJS_NAMES),\
		$(call generate_object,,$(OBJ_NAME)))


problems:
	$(foreach p,$(PROBLEMS_NAMES),\
		$(call generate_exec,,$(p)))

CL_OBJS = $(PROBLEMS_NAMES)
# CL_OBJS += $(AUX_OBJS_NAMES)

define clean_f

	-rm $(addsuffix .o,$($(1)))
	echo "Objects Deleted: $(addsuffix .o,$($(1)))\n"

endef

clean:
	@-rm $(addsuffix .o,$(CL_OBJS))
	@echo "Objects Deleted: $(addsuffix .o,$(CL_OBJS))\n"

complete:
	@echo "------------------\nOperation Summary:\n------------------\n"
	@-rm $(addsuffix .o,$(CL_OBJS))
	@echo "Objects Deleted: $(addsuffix .o,$(CL_OBJS))\n"
	@echo "Binaries successfully generated: $(PROBLEMS_NAMES)\n"