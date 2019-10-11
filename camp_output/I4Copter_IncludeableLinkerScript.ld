/* tc_link.ld -- Linker Scrift for a tricore
 * This file was generated automatically by the MMHT, do not 
 * modify this manually!
 */

OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH(tricore)
ENTRY(_start)


MEMORY
{
    
    PFlash0 wrxp: ORIGIN = 0x80000000, LENGTH = 2147483648
    
    PFlash1 wrxp: ORIGIN = 0x80200000, LENGTH = 2147483648
    
    DFlash0 wrxp: ORIGIN = 0xaf000000, LENGTH = 1073741824
    
    SRAM wrxp: ORIGIN = 0x90000000, LENGTH = 33554432
    
    CPU0_DSPR wrxp: ORIGIN = 0x70000000, LENGTH = 117440512
    
    CPU1_DSPR wrxp: ORIGIN = 0x60000000, LENGTH = 125829120
    
    CPU2_DSPR wrxp: ORIGIN = 0x50000000, LENGTH = 125829120
    
}

SECTIONS
{
    
    /* Sections for: PFlash0 */
    
    
    /* Sections for: PFlash1 */
    
    
    /* Sections for: DFlash0 */
    
    
    /* Sections for: SRAM */
    
    
    /* Sections for: CPU0_DSPR */
    
        
    .SystemApplication_OsCore_Core0_-1_IdleTask_OsCore_Core0_6 : ALIGN(64) {
        _start_SystemApplication_OsCore_Core0_-1_IdleTask_OsCore_Core0_6 = .;
        *(.SystemApplication_OsCore_Core0_-1_IdleTask_OsCore_Core0_6)
        _end_SystemApplication_OsCore_Core0_-1_IdleTask_OsCore_Core0_6 = .;
    } > CPU0_DSPR AT> TODO
        
    .SystemApplication_OsCore_Core2_-1_XSignalIsr_OsCore_Core2_8 : ALIGN(64) {
        _start_SystemApplication_OsCore_Core2_-1_XSignalIsr_OsCore_Core2_8 = .;
        *(.SystemApplication_OsCore_Core2_-1_XSignalIsr_OsCore_Core2_8)
        _end_SystemApplication_OsCore_Core2_-1_XSignalIsr_OsCore_Core2_8 = .;
    } > CPU0_DSPR AT> TODO
        
    .OS_App1_Core2_0_T5_AltitudeObserver_0 : ALIGN(64) {
        _start_OS_App1_Core2_0_T5_AltitudeObserver_0 = .;
        *(.OS_App1_Core2_0_T5_AltitudeObserver_0)
        _end_OS_App1_Core2_0_T5_AltitudeObserver_0 = .;
    } > CPU0_DSPR AT> TODO
        
    .OS_App0_Core2_-1_T6_DSP_6 : ALIGN(64) {
        _start_OS_App0_Core2_-1_T6_DSP_6 = .;
        *(.OS_App0_Core2_-1_T6_DSP_6)
        _end_OS_App0_Core2_-1_T6_DSP_6 = .;
    } > CPU0_DSPR AT> TODO
        
    .OS_App0_Core2_0_T6_DSP_0 : ALIGN(64) {
        _start_OS_App0_Core2_0_T6_DSP_0 = .;
        *(.OS_App0_Core2_0_T6_DSP_0)
        _end_OS_App0_Core2_0_T6_DSP_0 = .;
    } > CPU0_DSPR AT> TODO
        
    .OS_App0_Core2_0_T6_DSP_2 : ALIGN(64) {
        _start_OS_App0_Core2_0_T6_DSP_2 = .;
        *(.OS_App0_Core2_0_T6_DSP_2)
        _end_OS_App0_Core2_0_T6_DSP_2 = .;
    } > CPU0_DSPR AT> TODO
        
    .OS_App1_Core2_0_T5_AltitudeObserver_8 : ALIGN(64) {
        _start_OS_App1_Core2_0_T5_AltitudeObserver_8 = .;
        *(.OS_App1_Core2_0_T5_AltitudeObserver_8)
        _end_OS_App1_Core2_0_T5_AltitudeObserver_8 = .;
    } > CPU0_DSPR AT> TODO
        
    
    
    /* Sections for: CPU1_DSPR */
    
        
    .OS_App0_Core0_1_T1_Controllers_0 : ALIGN(64) {
        _start_OS_App0_Core0_1_T1_Controllers_0 = .;
        *(.OS_App0_Core0_1_T1_Controllers_0)
        _end_OS_App0_Core0_1_T1_Controllers_0 = .;
    } > CPU1_DSPR AT> TODO
        
    .OS_App0_Core0_1_T1_Controllers_8 : ALIGN(64) {
        _start_OS_App0_Core0_1_T1_Controllers_8 = .;
        *(.OS_App0_Core0_1_T1_Controllers_8)
        _end_OS_App0_Core0_1_T1_Controllers_8 = .;
    } > CPU1_DSPR AT> TODO
        
    .OS_App0_Core1_1_T2_EngineController_0 : ALIGN(64) {
        _start_OS_App0_Core1_1_T2_EngineController_0 = .;
        *(.OS_App0_Core1_1_T2_EngineController_0)
        _end_OS_App0_Core1_1_T2_EngineController_0 = .;
    } > CPU1_DSPR AT> TODO
        
    .OS_App0_Core1_1_T2_EngineController_8 : ALIGN(64) {
        _start_OS_App0_Core1_1_T2_EngineController_8 = .;
        *(.OS_App0_Core1_1_T2_EngineController_8)
        _end_OS_App0_Core1_1_T2_EngineController_8 = .;
    } > CPU1_DSPR AT> TODO
        
    .OS_App1_Core0_1_T3_AttitudeObserver_0 : ALIGN(64) {
        _start_OS_App1_Core0_1_T3_AttitudeObserver_0 = .;
        *(.OS_App1_Core0_1_T3_AttitudeObserver_0)
        _end_OS_App1_Core0_1_T3_AttitudeObserver_0 = .;
    } > CPU1_DSPR AT> TODO
        
    .OS_App1_Core0_1_T3_AttitudeObserver_8 : ALIGN(64) {
        _start_OS_App1_Core0_1_T3_AttitudeObserver_8 = .;
        *(.OS_App1_Core0_1_T3_AttitudeObserver_8)
        _end_OS_App1_Core0_1_T3_AttitudeObserver_8 = .;
    } > CPU1_DSPR AT> TODO
        
    
    
    /* Sections for: CPU2_DSPR */
    
        
    .OS_App1_Core1_2_T4_HeightObserver_0 : ALIGN(64) {
        _start_OS_App1_Core1_2_T4_HeightObserver_0 = .;
        *(.OS_App1_Core1_2_T4_HeightObserver_0)
        _end_OS_App1_Core1_2_T4_HeightObserver_0 = .;
    } > CPU2_DSPR AT> TODO
        
    .OS_App1_Core2_-1_T5_AltitudeObserver_8 : ALIGN(64) {
        _start_OS_App1_Core2_-1_T5_AltitudeObserver_8 = .;
        *(.OS_App1_Core2_-1_T5_AltitudeObserver_8)
        _end_OS_App1_Core2_-1_T5_AltitudeObserver_8 = .;
    } > CPU2_DSPR AT> TODO
        
    .OS_App1_Core1_2_T4_HeightObserver_8 : ALIGN(64) {
        _start_OS_App1_Core1_2_T4_HeightObserver_8 = .;
        *(.OS_App1_Core1_2_T4_HeightObserver_8)
        _end_OS_App1_Core1_2_T4_HeightObserver_8 = .;
    } > CPU2_DSPR AT> TODO
        
    
    

    /* Catch all sections */
    .everything {
        *(.text)
        *(.data)
        *(.bss)
        *(.rodata)
    } > TODO

    /* DWARF debug section */
    .comment         0 : { *(.comment) }
	.debug           0 : { *(.debug) }
	.line            0 : { *(.line) }
	/*
	 * GNU DWARF 1 extensions
	 */
	.debug_srcinfo   0 : { *(.debug_srcinfo) }
	.debug_sfnames   0 : { *(.debug_sfnames) }
	/*
	 * DWARF 1.1 and DWARF 2
	 */
	.debug_aranges   0 : { *(.debug_aranges) }
	.debug_pubnames  0 : { *(.debug_pubnames) }
	/*
	 * DWARF 2
	 */
	.debug_info      0 : { *(.debug_info) }
	.debug_abbrev    0 : { *(.debug_abbrev) }
	.debug_line      0 : { *(.debug_line) }
	.debug_frame     0 : { *(.debug_frame) }
	.debug_str       0 : { *(.debug_str) }
	.debug_loc       0 : { *(.debug_loc) }
	.debug_macinfo   0 : { *(.debug_macinfo) }
	.debug_ranges    0 : { *(.debug_ranges) }
	/*
	 * SGI/MIPS DWARF 2 extensions
	 */
	.debug_weaknames 0 : { *(.debug_weaknames) }
	.debug_funcnames 0 : { *(.debug_funcnames) }
	.debug_typenames 0 : { *(.debug_typenames) }
	.debug_varnames  0 : { *(.debug_varnames) }
	/*
	 * Optional sections that may only appear when relocating.
	 */
	/*
	 * Optional sections that may appear regardless of relocating.
	 */
	.version_info    0 : { *(.version_info) }
	.boffs           0 : { KEEP (*(.boffs)) }
}