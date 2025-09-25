SECTIONS
{
	.param : {
        PROVIDE(_stext = .);
        PROVIDE(_param_start = .);
        KEEP(*(.param))
        KEEP(*(.param.*))
        PROVIDE(_param_stop = .);
        PROVIDE(_etext = .);
    }
    .log : {
        PROVIDE(_log_start = .);
        KEEP(*(.log))
        KEEP(*(.log.*))
        PROVIDE(_log_stop = .);
    }
    .eventtrigger : {
        PROVIDE(_eventtrigger_start = .);
        KEEP(*(.eventtrigger));
        KEEP(*(.eventtrigger.*));
        PROVIDE(_eventtrigger_stop = .);
    }
    .data : {
        PROVIDE(_sidata = .);
        PROVIDE(_sdata = .);
        PROVIDE(_edata = .);
    }
     
}
INSERT AFTER .text;