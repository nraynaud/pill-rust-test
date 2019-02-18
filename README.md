# Blue Pill Balancing Robot

> A personal project to learn embedded rust on a STM32F103.

# Debugging
 - run gdb with `remote target | openocd -d1 -f openocd.cfg -c "gdb_port pipe" ` 
 it avoids you having to maintain a separated openocd process
 - in RTFM the idle task uses the wfi instruction. 
 When the MCU is waiting for an interruption, it 
 won't answer to debug unless you set the `dbg_sleep` at 1
 `DBG.cr`:
 ```rust
    #[init]
    unsafe fn init() {
        let dbg = device.DBG;
        dbg.cr.modify(|_, w| w.dbg_sleep().set_bit());
    }
``` 
- if using semihosting, you might want to use my 
[openocd version](https://github.com/nraynaud/openocd-semihosting-to-gdb)
