# SPDX-License-Identifier: EPICS
# Helmholtz-Zentrum Berlin fuer Materialien und Energie GmbH 2023-2024
# Lutz Rossa <rossa@helmholtz-berlin.de>

# Makefile at top of application tree
TOP = ..
include $(TOP)/configure/CONFIG

# Directories to be built, in any order.
# You can replace these wildcards with an explicit list
DIRS += $(wildcard src* *Src*)
DIRS += $(wildcard db* *Db*)

# If the build order matters, add dependency rules like this,
# which specifies that xxxSrc must be built after src:
#xxxSrc_DEPEND_DIRS += src

include $(TOP)/configure/RULES_DIRS
