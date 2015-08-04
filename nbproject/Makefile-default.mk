#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/e220.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/e220.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=light_link.c ll_vector.c ringbufs.c eadog.c crc8_fast.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/light_link.o ${OBJECTDIR}/ll_vector.o ${OBJECTDIR}/ringbufs.o ${OBJECTDIR}/eadog.o ${OBJECTDIR}/crc8_fast.o
POSSIBLE_DEPFILES=${OBJECTDIR}/light_link.o.d ${OBJECTDIR}/ll_vector.o.d ${OBJECTDIR}/ringbufs.o.d ${OBJECTDIR}/eadog.o.d ${OBJECTDIR}/crc8_fast.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/light_link.o ${OBJECTDIR}/ll_vector.o ${OBJECTDIR}/ringbufs.o ${OBJECTDIR}/eadog.o ${OBJECTDIR}/crc8_fast.o

# Source Files
SOURCEFILES=light_link.c ll_vector.c ringbufs.c eadog.c crc8_fast.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/e220.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F45K80
MP_PROCESSOR_OPTION_LD=18f45k80
MP_LINKER_DEBUG_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/light_link.o: light_link.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/light_link.o.d 
	@${RM} ${OBJECTDIR}/light_link.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/light_link.o   light_link.c 
	@${DEP_GEN} -d ${OBJECTDIR}/light_link.o 
	@${FIXDEPS} "${OBJECTDIR}/light_link.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/ll_vector.o: ll_vector.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ll_vector.o.d 
	@${RM} ${OBJECTDIR}/ll_vector.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/ll_vector.o   ll_vector.c 
	@${DEP_GEN} -d ${OBJECTDIR}/ll_vector.o 
	@${FIXDEPS} "${OBJECTDIR}/ll_vector.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/ringbufs.o: ringbufs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ringbufs.o.d 
	@${RM} ${OBJECTDIR}/ringbufs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/ringbufs.o   ringbufs.c 
	@${DEP_GEN} -d ${OBJECTDIR}/ringbufs.o 
	@${FIXDEPS} "${OBJECTDIR}/ringbufs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/eadog.o: eadog.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eadog.o.d 
	@${RM} ${OBJECTDIR}/eadog.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/eadog.o   eadog.c 
	@${DEP_GEN} -d ${OBJECTDIR}/eadog.o 
	@${FIXDEPS} "${OBJECTDIR}/eadog.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/crc8_fast.o: crc8_fast.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/crc8_fast.o.d 
	@${RM} ${OBJECTDIR}/crc8_fast.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/crc8_fast.o   crc8_fast.c 
	@${DEP_GEN} -d ${OBJECTDIR}/crc8_fast.o 
	@${FIXDEPS} "${OBJECTDIR}/crc8_fast.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
else
${OBJECTDIR}/light_link.o: light_link.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/light_link.o.d 
	@${RM} ${OBJECTDIR}/light_link.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/light_link.o   light_link.c 
	@${DEP_GEN} -d ${OBJECTDIR}/light_link.o 
	@${FIXDEPS} "${OBJECTDIR}/light_link.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/ll_vector.o: ll_vector.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ll_vector.o.d 
	@${RM} ${OBJECTDIR}/ll_vector.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/ll_vector.o   ll_vector.c 
	@${DEP_GEN} -d ${OBJECTDIR}/ll_vector.o 
	@${FIXDEPS} "${OBJECTDIR}/ll_vector.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/ringbufs.o: ringbufs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ringbufs.o.d 
	@${RM} ${OBJECTDIR}/ringbufs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/ringbufs.o   ringbufs.c 
	@${DEP_GEN} -d ${OBJECTDIR}/ringbufs.o 
	@${FIXDEPS} "${OBJECTDIR}/ringbufs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/eadog.o: eadog.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eadog.o.d 
	@${RM} ${OBJECTDIR}/eadog.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/eadog.o   eadog.c 
	@${DEP_GEN} -d ${OBJECTDIR}/eadog.o 
	@${FIXDEPS} "${OBJECTDIR}/eadog.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/crc8_fast.o: crc8_fast.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/crc8_fast.o.d 
	@${RM} ${OBJECTDIR}/crc8_fast.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -ml --extended -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/crc8_fast.o   crc8_fast.c 
	@${DEP_GEN} -d ${OBJECTDIR}/crc8_fast.o 
	@${FIXDEPS} "${OBJECTDIR}/crc8_fast.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/e220.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE)   -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG -m"${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" -u_EXTENDEDMODE -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_ICD3=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}/../lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/e220.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/e220.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE)   -p$(MP_PROCESSOR_OPTION_LD)  -w  -m"${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" -u_EXTENDEDMODE -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}/../lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/e220.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
