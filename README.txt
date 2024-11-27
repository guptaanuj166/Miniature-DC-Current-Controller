User manual:-  The document contains detailed info specifications, terminologies definitions, and instructions to use for the user along with additional instructions the user should take care of.

Design_documentation:- The document contains an overview of the project with the circuit as well as a block diagram. It also provides an overview of the CC-CV modes and control mechanisms.

PCB:- The directory contains .sch files which include schematic files and .pcb has placement and routing of components in the circuit.

CAD:- The directory contains all files for the final CAD assembly model of the project. It has mounting holes for wall attachment; each surface has groves for fitting a removable back panel structure.

Microcontroller_code:- The directory contains all code files of STM32 uC. The code contains a control algorithm for current, CC-CV mode transition, code for rotary encoder, hall-based current sensor to sense the current and setting reference voltage corresponding to current interfaces with DAC.

Test results:- The document includes test methodologies for the basic circuit, microcontroller integration, CC-CV switching and UART interfacing and the results of each methodology.

Frontend:- This contains python code for the web based GUI for UART control.