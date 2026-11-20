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
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/MCU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/MCU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=FrameworkSource/ES_CheckEvents.c FrameworkSource/ES_DeferRecall.c FrameworkSource/ES_Framework.c FrameworkSource/ES_LookupTables.c FrameworkSource/ES_Port.c FrameworkSource/ES_PostList.c FrameworkSource/ES_Queue.c FrameworkSource/ES_Timers.c FrameworkSource/terminal.c FrameworkSource/dbprintf.c FrameworkSource/circular_buffer_no_modulo_threadsafe.c ProjectSource/EventCheckers.c ProjectSource/main.c ProjectSource/IMU_SM.c ProjectSource/UsbService.c ProjectSource/MotorSM.c ProjectSource/JetsonSM.c ProjectSource/Button1DebouncerSM.c ProjectSource/Button2DebouncerSM.c ProjectSource/Button3DebouncerSM.c ProjectSource/LEDService.c ProjectSource/EEPROMSM.c ProjectSource/ReflectService.c ProjectSource/ADC_HAL.c ProjectSource/EnvironmentSensorSM.c ProjectSource/SDCardSM.c ProjectSource/sensirion_gas_index_algorithm.c ProjectSource/matt_circular_buffer.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o ${OBJECTDIR}/FrameworkSource/ES_Framework.o ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o ${OBJECTDIR}/FrameworkSource/ES_Port.o ${OBJECTDIR}/FrameworkSource/ES_PostList.o ${OBJECTDIR}/FrameworkSource/ES_Queue.o ${OBJECTDIR}/FrameworkSource/ES_Timers.o ${OBJECTDIR}/FrameworkSource/terminal.o ${OBJECTDIR}/FrameworkSource/dbprintf.o ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o ${OBJECTDIR}/ProjectSource/EventCheckers.o ${OBJECTDIR}/ProjectSource/main.o ${OBJECTDIR}/ProjectSource/IMU_SM.o ${OBJECTDIR}/ProjectSource/UsbService.o ${OBJECTDIR}/ProjectSource/MotorSM.o ${OBJECTDIR}/ProjectSource/JetsonSM.o ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o ${OBJECTDIR}/ProjectSource/LEDService.o ${OBJECTDIR}/ProjectSource/EEPROMSM.o ${OBJECTDIR}/ProjectSource/ReflectService.o ${OBJECTDIR}/ProjectSource/ADC_HAL.o ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o ${OBJECTDIR}/ProjectSource/SDCardSM.o ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o
POSSIBLE_DEPFILES=${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o.d ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o.d ${OBJECTDIR}/FrameworkSource/ES_Framework.o.d ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o.d ${OBJECTDIR}/FrameworkSource/ES_Port.o.d ${OBJECTDIR}/FrameworkSource/ES_PostList.o.d ${OBJECTDIR}/FrameworkSource/ES_Queue.o.d ${OBJECTDIR}/FrameworkSource/ES_Timers.o.d ${OBJECTDIR}/FrameworkSource/terminal.o.d ${OBJECTDIR}/FrameworkSource/dbprintf.o.d ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o.d ${OBJECTDIR}/ProjectSource/EventCheckers.o.d ${OBJECTDIR}/ProjectSource/main.o.d ${OBJECTDIR}/ProjectSource/IMU_SM.o.d ${OBJECTDIR}/ProjectSource/UsbService.o.d ${OBJECTDIR}/ProjectSource/MotorSM.o.d ${OBJECTDIR}/ProjectSource/JetsonSM.o.d ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o.d ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o.d ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o.d ${OBJECTDIR}/ProjectSource/LEDService.o.d ${OBJECTDIR}/ProjectSource/EEPROMSM.o.d ${OBJECTDIR}/ProjectSource/ReflectService.o.d ${OBJECTDIR}/ProjectSource/ADC_HAL.o.d ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o.d ${OBJECTDIR}/ProjectSource/SDCardSM.o.d ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o.d ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o ${OBJECTDIR}/FrameworkSource/ES_Framework.o ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o ${OBJECTDIR}/FrameworkSource/ES_Port.o ${OBJECTDIR}/FrameworkSource/ES_PostList.o ${OBJECTDIR}/FrameworkSource/ES_Queue.o ${OBJECTDIR}/FrameworkSource/ES_Timers.o ${OBJECTDIR}/FrameworkSource/terminal.o ${OBJECTDIR}/FrameworkSource/dbprintf.o ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o ${OBJECTDIR}/ProjectSource/EventCheckers.o ${OBJECTDIR}/ProjectSource/main.o ${OBJECTDIR}/ProjectSource/IMU_SM.o ${OBJECTDIR}/ProjectSource/UsbService.o ${OBJECTDIR}/ProjectSource/MotorSM.o ${OBJECTDIR}/ProjectSource/JetsonSM.o ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o ${OBJECTDIR}/ProjectSource/LEDService.o ${OBJECTDIR}/ProjectSource/EEPROMSM.o ${OBJECTDIR}/ProjectSource/ReflectService.o ${OBJECTDIR}/ProjectSource/ADC_HAL.o ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o ${OBJECTDIR}/ProjectSource/SDCardSM.o ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o

# Source Files
SOURCEFILES=FrameworkSource/ES_CheckEvents.c FrameworkSource/ES_DeferRecall.c FrameworkSource/ES_Framework.c FrameworkSource/ES_LookupTables.c FrameworkSource/ES_Port.c FrameworkSource/ES_PostList.c FrameworkSource/ES_Queue.c FrameworkSource/ES_Timers.c FrameworkSource/terminal.c FrameworkSource/dbprintf.c FrameworkSource/circular_buffer_no_modulo_threadsafe.c ProjectSource/EventCheckers.c ProjectSource/main.c ProjectSource/IMU_SM.c ProjectSource/UsbService.c ProjectSource/MotorSM.c ProjectSource/JetsonSM.c ProjectSource/Button1DebouncerSM.c ProjectSource/Button2DebouncerSM.c ProjectSource/Button3DebouncerSM.c ProjectSource/LEDService.c ProjectSource/EEPROMSM.c ProjectSource/ReflectService.c ProjectSource/ADC_HAL.c ProjectSource/EnvironmentSensorSM.c ProjectSource/SDCardSM.c ProjectSource/sensirion_gas_index_algorithm.c ProjectSource/matt_circular_buffer.c



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
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/MCU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048EFH144
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o: FrameworkSource/ES_CheckEvents.c  .generated_files/flags/default/51ed3176bad6cf38f26072ae47486c8de26c66fa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o FrameworkSource/ES_CheckEvents.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o: FrameworkSource/ES_DeferRecall.c  .generated_files/flags/default/ed07e818332dd8479e4672dcd400b6b91ceefe9c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o FrameworkSource/ES_DeferRecall.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_Framework.o: FrameworkSource/ES_Framework.c  .generated_files/flags/default/1af3ba1ffd2a119da163af824d6ae70068596f1d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Framework.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Framework.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_Framework.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_Framework.o FrameworkSource/ES_Framework.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_LookupTables.o: FrameworkSource/ES_LookupTables.c  .generated_files/flags/default/45ead179739bd3bd9f47c862660ec9117d6b8cc7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_LookupTables.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o FrameworkSource/ES_LookupTables.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_Port.o: FrameworkSource/ES_Port.c  .generated_files/flags/default/5619451e3bb4e292c03a32741fd30e24a4c4b6d2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Port.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_Port.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_Port.o FrameworkSource/ES_Port.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_PostList.o: FrameworkSource/ES_PostList.c  .generated_files/flags/default/c8a50583385a5cf7b0917df51fd5c4558f29592b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_PostList.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_PostList.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_PostList.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_PostList.o FrameworkSource/ES_PostList.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_Queue.o: FrameworkSource/ES_Queue.c  .generated_files/flags/default/dedcf6aaccbe353cab7a58f1dfc5212f21c156b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Queue.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_Queue.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_Queue.o FrameworkSource/ES_Queue.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_Timers.o: FrameworkSource/ES_Timers.c  .generated_files/flags/default/b2c55f7d4e58229f8a24d80a0280f6216d6a9b05 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Timers.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_Timers.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_Timers.o FrameworkSource/ES_Timers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/terminal.o: FrameworkSource/terminal.c  .generated_files/flags/default/a1b678a7033a1586acf4547354baeb3e1866dfc8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/terminal.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/terminal.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/terminal.o.d" -o ${OBJECTDIR}/FrameworkSource/terminal.o FrameworkSource/terminal.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/dbprintf.o: FrameworkSource/dbprintf.c  .generated_files/flags/default/6ac2bf6bd63ed6919985df2051ab554230573e9d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/dbprintf.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/dbprintf.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/dbprintf.o.d" -o ${OBJECTDIR}/FrameworkSource/dbprintf.o FrameworkSource/dbprintf.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o: FrameworkSource/circular_buffer_no_modulo_threadsafe.c  .generated_files/flags/default/3e76ed140498cdc0e66c3838bf6c697908fb8dfc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o.d" -o ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o FrameworkSource/circular_buffer_no_modulo_threadsafe.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/EventCheckers.o: ProjectSource/EventCheckers.c  .generated_files/flags/default/7556c0dd5d46c8a0e3d8ab5ed755505392600be1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/EventCheckers.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/EventCheckers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/EventCheckers.o.d" -o ${OBJECTDIR}/ProjectSource/EventCheckers.o ProjectSource/EventCheckers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/main.o: ProjectSource/main.c  .generated_files/flags/default/79af837934c0acebce66b58216e08fec4923dd3d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/main.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/main.o.d" -o ${OBJECTDIR}/ProjectSource/main.o ProjectSource/main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/IMU_SM.o: ProjectSource/IMU_SM.c  .generated_files/flags/default/abe321212b6543e5c97aba6016eab505fc04dd53 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/IMU_SM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/IMU_SM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/IMU_SM.o.d" -o ${OBJECTDIR}/ProjectSource/IMU_SM.o ProjectSource/IMU_SM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/UsbService.o: ProjectSource/UsbService.c  .generated_files/flags/default/24311360c891c464b76b17144cc6f89462e48bb3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/UsbService.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/UsbService.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/UsbService.o.d" -o ${OBJECTDIR}/ProjectSource/UsbService.o ProjectSource/UsbService.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/MotorSM.o: ProjectSource/MotorSM.c  .generated_files/flags/default/1059de8309a45ccda8153d0b11e67a2ff2b5e4c5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/MotorSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/MotorSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/MotorSM.o.d" -o ${OBJECTDIR}/ProjectSource/MotorSM.o ProjectSource/MotorSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/JetsonSM.o: ProjectSource/JetsonSM.c  .generated_files/flags/default/ee75c204f23329f71951be86fedb663f30ed0f04 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/JetsonSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/JetsonSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/JetsonSM.o.d" -o ${OBJECTDIR}/ProjectSource/JetsonSM.o ProjectSource/JetsonSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o: ProjectSource/Button1DebouncerSM.c  .generated_files/flags/default/a2b2510cbda4d08c463c67c8260e7ee58f5cc03d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o.d" -o ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o ProjectSource/Button1DebouncerSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o: ProjectSource/Button2DebouncerSM.c  .generated_files/flags/default/102b228fe0e22fca47e35212eaf6dcc13d66f155 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o.d" -o ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o ProjectSource/Button2DebouncerSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o: ProjectSource/Button3DebouncerSM.c  .generated_files/flags/default/e27b51e1164dbe19f9c18fb40d86aff90557f2cb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o.d" -o ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o ProjectSource/Button3DebouncerSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/LEDService.o: ProjectSource/LEDService.c  .generated_files/flags/default/6fe6028a367f0fc5437497f49c0d209d11c1f058 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/LEDService.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/LEDService.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/LEDService.o.d" -o ${OBJECTDIR}/ProjectSource/LEDService.o ProjectSource/LEDService.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/EEPROMSM.o: ProjectSource/EEPROMSM.c  .generated_files/flags/default/81016c69fac17cd803fbc78d4ed16fc4ea632068 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/EEPROMSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/EEPROMSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/EEPROMSM.o.d" -o ${OBJECTDIR}/ProjectSource/EEPROMSM.o ProjectSource/EEPROMSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/ReflectService.o: ProjectSource/ReflectService.c  .generated_files/flags/default/cf8834c4e94241d58007a8edf5b6f4e9351fd66a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/ReflectService.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/ReflectService.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/ReflectService.o.d" -o ${OBJECTDIR}/ProjectSource/ReflectService.o ProjectSource/ReflectService.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/ADC_HAL.o: ProjectSource/ADC_HAL.c  .generated_files/flags/default/8fabd57f1d8e5a610bb815dd6c28b82e6024ce40 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/ADC_HAL.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/ADC_HAL.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/ADC_HAL.o.d" -o ${OBJECTDIR}/ProjectSource/ADC_HAL.o ProjectSource/ADC_HAL.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o: ProjectSource/EnvironmentSensorSM.c  .generated_files/flags/default/6b875f7d0ec5f653db890f6810a11c0d759a50fa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o.d" -o ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o ProjectSource/EnvironmentSensorSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/SDCardSM.o: ProjectSource/SDCardSM.c  .generated_files/flags/default/20a44454f8786e5db3ba015dac33748f5be717f1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/SDCardSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/SDCardSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/SDCardSM.o.d" -o ${OBJECTDIR}/ProjectSource/SDCardSM.o ProjectSource/SDCardSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o: ProjectSource/sensirion_gas_index_algorithm.c  .generated_files/flags/default/4f940cb658a958fcd484139337be45cc47ee12f7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o.d" -o ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o ProjectSource/sensirion_gas_index_algorithm.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/matt_circular_buffer.o: ProjectSource/matt_circular_buffer.c  .generated_files/flags/default/1e12dbd5513f0af0a398b36dec89c8b9d2d26587 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_SNAP=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/matt_circular_buffer.o.d" -o ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o ProjectSource/matt_circular_buffer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o: FrameworkSource/ES_CheckEvents.c  .generated_files/flags/default/d88b6eb392944e40c8af5fea34e3bb821616d5ac .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_CheckEvents.o FrameworkSource/ES_CheckEvents.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o: FrameworkSource/ES_DeferRecall.c  .generated_files/flags/default/56d89ca5f49d3d64243f8120455210936cf4f712 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_DeferRecall.o FrameworkSource/ES_DeferRecall.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_Framework.o: FrameworkSource/ES_Framework.c  .generated_files/flags/default/8ff350e6a378152365b310f2dba6544e3f53843 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Framework.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Framework.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_Framework.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_Framework.o FrameworkSource/ES_Framework.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_LookupTables.o: FrameworkSource/ES_LookupTables.c  .generated_files/flags/default/cc5ddb8dc0ece807230bec8c3b4f3cc383ccb30b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_LookupTables.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_LookupTables.o FrameworkSource/ES_LookupTables.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_Port.o: FrameworkSource/ES_Port.c  .generated_files/flags/default/50cb9bd6c7e5c802ed64b58116a5dab8c52c85ce .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Port.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_Port.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_Port.o FrameworkSource/ES_Port.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_PostList.o: FrameworkSource/ES_PostList.c  .generated_files/flags/default/50978610e9d0ebb37c9ca3a116520caa592a8982 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_PostList.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_PostList.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_PostList.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_PostList.o FrameworkSource/ES_PostList.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_Queue.o: FrameworkSource/ES_Queue.c  .generated_files/flags/default/45d89a2354239de630753760dedb00ce87808b18 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Queue.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_Queue.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_Queue.o FrameworkSource/ES_Queue.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/ES_Timers.o: FrameworkSource/ES_Timers.c  .generated_files/flags/default/f84ca0aa4531ac1fd320c2dd17b535e72a044291 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Timers.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/ES_Timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/ES_Timers.o.d" -o ${OBJECTDIR}/FrameworkSource/ES_Timers.o FrameworkSource/ES_Timers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/terminal.o: FrameworkSource/terminal.c  .generated_files/flags/default/6b293599e0b25535e10b8a5db43e88794b86f103 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/terminal.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/terminal.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/terminal.o.d" -o ${OBJECTDIR}/FrameworkSource/terminal.o FrameworkSource/terminal.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/dbprintf.o: FrameworkSource/dbprintf.c  .generated_files/flags/default/acf1ed831b3e54eb0c60c4d22e212b3bf5da24de .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/dbprintf.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/dbprintf.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/dbprintf.o.d" -o ${OBJECTDIR}/FrameworkSource/dbprintf.o FrameworkSource/dbprintf.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o: FrameworkSource/circular_buffer_no_modulo_threadsafe.c  .generated_files/flags/default/5b0180eac9d80528ca3bf3d48a793f7d268d85ee .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/FrameworkSource" 
	@${RM} ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o.d 
	@${RM} ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o.d" -o ${OBJECTDIR}/FrameworkSource/circular_buffer_no_modulo_threadsafe.o FrameworkSource/circular_buffer_no_modulo_threadsafe.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/EventCheckers.o: ProjectSource/EventCheckers.c  .generated_files/flags/default/c54d5ce69aaa5a4f476690044be83fe1e9edc47e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/EventCheckers.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/EventCheckers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/EventCheckers.o.d" -o ${OBJECTDIR}/ProjectSource/EventCheckers.o ProjectSource/EventCheckers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/main.o: ProjectSource/main.c  .generated_files/flags/default/f6a77c75291dd72cacdd2db2b96979be1485a028 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/main.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/main.o.d" -o ${OBJECTDIR}/ProjectSource/main.o ProjectSource/main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/IMU_SM.o: ProjectSource/IMU_SM.c  .generated_files/flags/default/564beb191d53d80757c687dcb2b80a8503795a3c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/IMU_SM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/IMU_SM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/IMU_SM.o.d" -o ${OBJECTDIR}/ProjectSource/IMU_SM.o ProjectSource/IMU_SM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/UsbService.o: ProjectSource/UsbService.c  .generated_files/flags/default/8e89c78bbacf34b52da8de34d205bb20b737d4f6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/UsbService.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/UsbService.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/UsbService.o.d" -o ${OBJECTDIR}/ProjectSource/UsbService.o ProjectSource/UsbService.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/MotorSM.o: ProjectSource/MotorSM.c  .generated_files/flags/default/30f4169dded207dd220babe6c107d7431b872c13 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/MotorSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/MotorSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/MotorSM.o.d" -o ${OBJECTDIR}/ProjectSource/MotorSM.o ProjectSource/MotorSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/JetsonSM.o: ProjectSource/JetsonSM.c  .generated_files/flags/default/8e86af26c237e812771a20b10e10bf99b4a86c7e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/JetsonSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/JetsonSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/JetsonSM.o.d" -o ${OBJECTDIR}/ProjectSource/JetsonSM.o ProjectSource/JetsonSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o: ProjectSource/Button1DebouncerSM.c  .generated_files/flags/default/30afedaba63f08d7cf3e85e1170f2471e76ce9bf .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o.d" -o ${OBJECTDIR}/ProjectSource/Button1DebouncerSM.o ProjectSource/Button1DebouncerSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o: ProjectSource/Button2DebouncerSM.c  .generated_files/flags/default/448ddf1f542d3b9e1a20b76972a13f1d47ce5e28 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o.d" -o ${OBJECTDIR}/ProjectSource/Button2DebouncerSM.o ProjectSource/Button2DebouncerSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o: ProjectSource/Button3DebouncerSM.c  .generated_files/flags/default/1f3c6c91f59344f3d28ea71597a45d6ad75d4f55 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o.d" -o ${OBJECTDIR}/ProjectSource/Button3DebouncerSM.o ProjectSource/Button3DebouncerSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/LEDService.o: ProjectSource/LEDService.c  .generated_files/flags/default/f138012456ce4ced54747b3738ab027f545fd71a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/LEDService.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/LEDService.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/LEDService.o.d" -o ${OBJECTDIR}/ProjectSource/LEDService.o ProjectSource/LEDService.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/EEPROMSM.o: ProjectSource/EEPROMSM.c  .generated_files/flags/default/d7946bcd3f92e1c37e8b8eaafa73d2ce41494be2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/EEPROMSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/EEPROMSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/EEPROMSM.o.d" -o ${OBJECTDIR}/ProjectSource/EEPROMSM.o ProjectSource/EEPROMSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/ReflectService.o: ProjectSource/ReflectService.c  .generated_files/flags/default/c41f9804da0ce8c2f80259e594265cb284cb09d9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/ReflectService.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/ReflectService.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/ReflectService.o.d" -o ${OBJECTDIR}/ProjectSource/ReflectService.o ProjectSource/ReflectService.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/ADC_HAL.o: ProjectSource/ADC_HAL.c  .generated_files/flags/default/9d573a86c81b70b5a38c1a9128095cf378f8c88c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/ADC_HAL.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/ADC_HAL.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/ADC_HAL.o.d" -o ${OBJECTDIR}/ProjectSource/ADC_HAL.o ProjectSource/ADC_HAL.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o: ProjectSource/EnvironmentSensorSM.c  .generated_files/flags/default/9193060f79703f19c0186aedb69e89ded769fb2d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o.d" -o ${OBJECTDIR}/ProjectSource/EnvironmentSensorSM.o ProjectSource/EnvironmentSensorSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/SDCardSM.o: ProjectSource/SDCardSM.c  .generated_files/flags/default/f47d0cadc9202c19e7ac49c1370ccebde3757221 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/SDCardSM.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/SDCardSM.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/SDCardSM.o.d" -o ${OBJECTDIR}/ProjectSource/SDCardSM.o ProjectSource/SDCardSM.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o: ProjectSource/sensirion_gas_index_algorithm.c  .generated_files/flags/default/71a43c7fcc9d2add64a3fca5fddc4e8b0a99f03d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o.d" -o ${OBJECTDIR}/ProjectSource/sensirion_gas_index_algorithm.o ProjectSource/sensirion_gas_index_algorithm.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/ProjectSource/matt_circular_buffer.o: ProjectSource/matt_circular_buffer.c  .generated_files/flags/default/9b3fa81a801fbf4da120e544a479555bcfc676c2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/ProjectSource" 
	@${RM} ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o.d 
	@${RM} ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"FrameworkHeaders" -I"ProjectHeaders" -fno-common -MP -MMD -MF "${OBJECTDIR}/ProjectSource/matt_circular_buffer.o.d" -o ${OBJECTDIR}/ProjectSource/matt_circular_buffer.o ProjectSource/matt_circular_buffer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/MCU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_SNAP=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o ${DISTDIR}/MCU.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x0:0x37F   -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_SNAP=1,--defsym=_min_heap_size=256,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,${DISTDIR}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
${DISTDIR}/MCU.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o ${DISTDIR}/MCU.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=256,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,${DISTDIR}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex ${DISTDIR}/MCU.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
