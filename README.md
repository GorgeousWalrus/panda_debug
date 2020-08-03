# debug_module

A debug module for cores
* Master debug module takes commands from an external source (to be changed to UART/JTAG)
* Master debug module executes commands:
  * Reset core
  * Reset peripherals
  * Reset everything
  * Read from wishbone slave
  * Write to wishbone slave
 * Master passes commands to slave debug module and reads back data
  * Core debug module can
    * Halt the core
    * Read from register file
    * Write to register file
    * Read current PC (from IF stage)
    * Set current PC (and flush the pipeline)
