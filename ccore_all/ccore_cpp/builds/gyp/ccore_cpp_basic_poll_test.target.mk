# This file is generated by gyp; do not edit.

TOOLSET := target
TARGET := ccore_cpp_basic_poll_test
DEFS_Default := \
	'-DCCORE_CPP_GYP_BUILD' \
	'-DCCORE_CPP_HAVE_LINUX' \
	'-DCCORE_STATIC' \
	'-DZMQ_STATIC' \
	'-DCZMQ_STATIC' \
	'-DBSON_STATIC' \
	'-DZMQ_BUILD_DRAFT_API' \
	'-DCZMQ_BUILD_DRAFT_API'

# Flags passed to all source files.
CFLAGS_Default :=

# Flags passed to only C files.
CFLAGS_C_Default :=

# Flags passed to only C++ files.
CFLAGS_CC_Default := \
	-std=c++11

INCS_Default := \
	-I../../../ccore_cpp/include \
	-I../../../ccore_cpp/builds/gyp \
	-I../../../ccore_lib/include \
	-I../../../libzmq/include \
	-I../../../czmq/include \
	-I../../../ccore_bson/ccore_bson/include \
	-I../../../libbson/src \
	-I../../../libbson/builds/gyp/linux

OBJS := \
	$(obj).target/$(TARGET)/../../../ccore_cpp/src/test/basic_poll.o

# Add to the list of files we specially track dependencies for.
all_deps += $(OBJS)

# Make sure our dependencies are built before any of us.
$(OBJS): | $(obj).target/../../../ccore_lib/builds/gyp/libccore.a $(obj).target/../../../ccore_bson/builds/gyp/libccore_bson.a $(obj).target/../../../libbson/builds/gyp/libbson.a $(obj).target/../../../ccore_cpp/builds/gyp/libccore_cpp.a $(obj).target/../../../libzmq/builds/gyp/libzmq.a $(obj).target/../../../czmq/builds/gyp/libczmq.a

# CFLAGS et al overrides must be target-local.
# See "Target-specific Variable Values" in the GNU Make manual.
$(OBJS): TOOLSET := $(TOOLSET)
$(OBJS): GYP_CFLAGS := $(DEFS_$(BUILDTYPE)) $(INCS_$(BUILDTYPE))  $(CFLAGS_$(BUILDTYPE)) $(CFLAGS_C_$(BUILDTYPE))
$(OBJS): GYP_CXXFLAGS := $(DEFS_$(BUILDTYPE)) $(INCS_$(BUILDTYPE))  $(CFLAGS_$(BUILDTYPE)) $(CFLAGS_CC_$(BUILDTYPE))

# Suffix rules, putting all outputs into $(obj).

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(srcdir)/%.cpp FORCE_DO_CMD
	@$(call do_cmd,cxx,1)

# Try building from generated source, too.

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(obj).$(TOOLSET)/%.cpp FORCE_DO_CMD
	@$(call do_cmd,cxx,1)

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(obj)/%.cpp FORCE_DO_CMD
	@$(call do_cmd,cxx,1)

# End of this set of suffix rules
### Rules for final target.
LDFLAGS_Default :=

LIBS := \
	-lpthread

$(builddir)/ccore_cpp_basic_poll_test: GYP_LDFLAGS := $(LDFLAGS_$(BUILDTYPE))
$(builddir)/ccore_cpp_basic_poll_test: LIBS := $(LIBS)
$(builddir)/ccore_cpp_basic_poll_test: LD_INPUTS := $(OBJS) $(obj).target/../../../ccore_lib/builds/gyp/libccore.a $(obj).target/../../../ccore_bson/builds/gyp/libccore_bson.a $(obj).target/../../../libbson/builds/gyp/libbson.a $(obj).target/../../../ccore_cpp/builds/gyp/libccore_cpp.a $(obj).target/../../../libzmq/builds/gyp/libzmq.a $(obj).target/../../../czmq/builds/gyp/libczmq.a
$(builddir)/ccore_cpp_basic_poll_test: TOOLSET := $(TOOLSET)
$(builddir)/ccore_cpp_basic_poll_test: $(OBJS) $(obj).target/../../../ccore_lib/builds/gyp/libccore.a $(obj).target/../../../ccore_bson/builds/gyp/libccore_bson.a $(obj).target/../../../libbson/builds/gyp/libbson.a $(obj).target/../../../ccore_cpp/builds/gyp/libccore_cpp.a $(obj).target/../../../libzmq/builds/gyp/libzmq.a $(obj).target/../../../czmq/builds/gyp/libczmq.a FORCE_DO_CMD
	$(call do_cmd,link)

all_deps += $(builddir)/ccore_cpp_basic_poll_test
# Add target alias
.PHONY: ccore_cpp_basic_poll_test
ccore_cpp_basic_poll_test: $(builddir)/ccore_cpp_basic_poll_test
