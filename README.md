Download the utility from [GitHub](https://github.com/DnCraptor/murmulator-tester/releases).  

- **m1p1** – for Murmulator 1.x Raspberry Pi Pico (or RP2040-compatible devices).  
- **m1p2** – for Murmulator 1.x Raspberry Pi Pico 2 (or RP2350-compatible devices).  
- **m2p1** – for Murmulator 2.0 Raspberry Pi Pico (or RP2040-compatible devices).  
- **m2p2** – for Murmulator 2.0 Raspberry Pi Pico 2 (or RP2350-compatible devices).  

### Important Notes:
- Pay attention to the **video output type** you are using. For example, the **HDMI version will not properly test VGA**, and vice versa.  
- If you have built a **multi-output board** (like Frank's design), prioritize **VGA output** for testing.  

### Expected Behavior After Flashing:
1. Two very short LED blinks on the Raspberry Pi board.  
2. If **long LED blinks occur afterward**, count them – they indicate the **GPIO number that has an (unwanted) connection with the previous GPIO**.  
   - There is a **1-second delay** between tests.  
   - If multiple connections exist, count the number of separate blinks.  
3. Afterward, four very short LED blinks occur, and the program attempts to start the video output.  
4. Further **on-screen output should be self-explanatory**.  

### Additional Notes:
- **Red text usually indicates an error**.  
- If **SDRAM is soldered and detected**, a **linear read/write test** is performed.  
- If **NES PAD[s] (joysticks) are connected**, button press codes appear in the bottom row.  
- If a **PS/2 keyboard is connected**, the second-to-last row shows the scan code of the last pressed key.  

### Sound Testing:
- **Press S, L, or R** to test **PWM audio**.  
- **Press I** to test **I²S audio**, then **L and/or R** to activate left/right channels.  
- Multiple keys can be pressed simultaneously to enable all channels.  

### Reset & Restart:
- Press **Ctrl+Alt+Del** (or the **Reset button** on the board, if available) to reboot.  
- If nothing works, **power cycle the board** (turn it off and on).
- Use custom overclocking: "+" or "-" key on NumPad changes CPU frequency by 4 MHz; "Ins" or "Del" key changes CPU frequency by 40 MHz.
