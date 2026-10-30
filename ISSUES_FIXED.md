# Issues fixed from the thesis prototype

1. **Caller ID dependency removed from control flow**  
   The thesis states that SIM900 did not offer a usable AT command to read the incoming call number, which became a major problem. This project uses SMS commands for arming/disarming and status control.

2. **`if(activated=true)` bug removed**  
   The appendix code contains assignment inside an `if` condition. The new sketch uses an explicit state machine.

3. **Blocking loop logic replaced**  
   The prototype code relied on delays and blocking sequences. The new sketch keeps GSM and GPS polling non-blocking.

4. **Hardware serial ports used properly on Mega**  
   The Mega 2560 has multiple hardware serial ports, so GSM and GPS are separated cleanly.

5. **Location request implemented end-to-end**  
   The thesis defines a `Location` command, but the appendix implementation was incomplete. This version replies with coordinates from parsed RMC sentences.

6. **Battery monitor added as a working feature**  
   The thesis defines automatic low battery detection; this project includes it with a configurable divider ratio.

7. **SMS whitelist added**  
   Only the configured owner number can control the system.
