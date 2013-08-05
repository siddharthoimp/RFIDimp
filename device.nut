// AS3911 Eval Board
// Pin 1: IRQ
// Pin 2: MISO
// Pin 5: SCLK
// Pin 7: MOSI
// Pin 8: CS_L
// Pin 9: unused
server.log("");
server.log("AS3911 Eval Board Started");

// Constants
// SPI Clock Frequency in kHz
// (over 2MHz may be unstable due to bug in AS3911, check errata document)
const FREQ = 1000;
// Receive types
const RECEIVE_TYPE_UNKNOWN = 0;
const RECEIVE_TYPE_ATQA = 1;
const RECEIVE_TYPE_ANTICOLLISION = 2;
const RECEIVE_TYPE_SAK = 3;

//Number of seconds to wait before we go to sleep
const SLEEP_TIME = 4;

class AS3911
{
  receiveType = null;      // Type of Received data
	atqa = null;			// Answer to REQA
	collision = null;		// Collision interrupt flag
	cardPresent = null;	    // Flag for detecting physical presence of tag (through cap sensor, amplitude measurement, etc.)
	UID = null;			    // Unique ID for RFID card
	cascadeLevel = null;	// Cascade level during select / anticollision process
	sleepTime = null; // Time counter before we sleep
  
	// Alias objects
	irq = null;	    // interrupt pin
	spi = null;	    // serial port
	cs_l = null;	// chip select

	constructor(interruptPin, serialPort, chipSelect){

		receiveType = 0;
		atqa = 0;
		collision = 0;        
		cardPresent = 0;
		UID = blob();
		cascadeLevel = 1;
    sleepTime = time() + SLEEP_TIME; 

		irq = interruptPin;
		spi = serialPort;
		cs_l = chipSelect;
	}

	// Read a register and return its value
	function regRead(addr) {
		addr = addr | 0x40;             // Set mode bits to Register Read
		cs_l.write(0);                  // Select AS3911
		spi.write(format("%c",addr));   // Write mode+address
		local reg = spi.readblob(1);    // Read byte from register
		cs_l.write(1);                  // Deselect AS3911
		
		return reg[0];                  // Return register value
	}

	// Read a register and log its value to the server
	function regPrint(addr) {
		server.log(format("Register 0x%02X: 0x%02X", addr, regRead(addr))); 
	}

	// Write to a register
	function regWrite(addr, byte) {
		// Mode bits for Register Write are '00', so no action required there
		cs_l.write(0);                          // Select AS3911
		spi.write(format("%c%c", addr, byte));  // Write address+data
		cs_l.write(1);                          // Deselect AS3911
	}

	// Set one bit in a register
	function setBit(reg, bitNum) {
		if (bitNum < 0 || bitNum > 7) {
			server.log("Error: Invalid bit #" + bitNum);
			return;
		}
		local newVal = regRead(reg) | 0x01 << bitNum;
		regWrite(reg, newVal);
	}

	// Clear one bit in a register
	function clearBit(reg, bitNum) {
		if (bitNum < 0 || bitNum > 7) {
			server.log("Error: Invalid bit #" + bitNum);
			return;
		}
		local newVal = regRead(reg) & ~(0x01 << bitNum);
		regWrite(reg, newVal);
	}

	// Send a direct command
	function directCommand(cmd) {
		// Direct Command mode bits are '11' and already included in command code
		cs_l.write(0);                  // Select AS3911
		spi.write(format("%c", cmd));   // Write command
		cs_l.write(1);                  // Deselect AS3911
	}

	// Read bytes from FIFO
	function FIFORead() {
		local bytesToRead = regRead(0x1A);  // Number of unread bytes in FIFO
		local FIFOStatusReg = regRead(0x1B);// 
//		server.log(format("FIFO REG1: 0x%02X", bytesToRead));
//		server.log(format("FIFO REG2: 0x%02X", FIFOStatusReg));
		
		cs_l.write(0);                      // Select AS3911
		spi.write(format("%c", 0xBF));      // Write "FIFO Read" mode bits
		local receivedFIFO = spi.readblob(bytesToRead);  // Read entire FIFO into a blob
		cs_l.write(1);                      // Deselect AS3911
		
		return receivedFIFO;
	}

	// Write bytes to FIFO
	function FIFOWrite(dataBlob) {
		if (dataBlob.len() > 96) {
			server.log("FIFO must be 96 bytes or less!");
			return;
		}
		cs_l.write(0);                      // Select AS3911
		spi.write(format("%c", 0x80));      // Write "FIFO Write" mode bits
		spi.write(dataBlob);                    // Write contents of FIFO
		cs_l.write(1);                      // Deselect AS3911
	}

	// Anticollision
	function anticollision(nvb, isSelect, isFinal) {
		local FIFOBlob = blob();
		local sel = 0x93 + (cascadeLevel - 1) * 2
		FIFOBlob.writen(sel, 'b');  // SEL
		FIFOBlob.writen(nvb, 'b');  // NVB
		
		if(isSelect) {
			clearBit(0x09, 7);      // Make sure no_CRC_rx is cleared
			clearBit(0x05, 0);      // Clear antcl
			receiveType = RECEIVE_TYPE_SAK; // Expect to receive SAK
			if (isFinal) {
				FIFOBlob.writen(UID[3], 'b');
				FIFOBlob.writen(UID[4], 'b');
				FIFOBlob.writen(UID[5], 'b');
				FIFOBlob.writen(UID[6], 'b');
				FIFOBlob.writen(UID[3] ^ UID[4] ^ UID[5] ^ UID[6], 'b');// BCC
			}
			else {
				FIFOBlob.writen(0x88, 'b');                             // CT
				FIFOBlob.writen(UID[0], 'b');
				FIFOBlob.writen(UID[1], 'b');
				FIFOBlob.writen(UID[2], 'b');
				FIFOBlob.writen(0x88 ^ UID[0] ^ UID[1] ^ UID[2], 'b');  // BCC
			}
			directCommand(0xC2);    // Clear FIFO / status registers
			regWrite(0x1D, FIFOBlob.len() >> 8);    // Write # of Transmitted Bytes (MSB)
			regWrite(0x1E, FIFOBlob.len() << 3);    // Write # of Transmitted Bytes (LSB)
			FIFOWrite(FIFOBlob);
			directCommand(0xC4);    //Transmit contents of FIFO with CRC
			server.log("Transmitting select packet:");
		} else {
			receiveType = RECEIVE_TYPE_ANTICOLLISION;   // Expect anticollision frame
			local length = (nvb >> 4) - FIFOBlob.len();
			server.log(format("NVB: %i, Length: %i, FIFO length: %i", nvb >> 4, length, FIFOBlob.len()));
			for (local i = 0; i < length && i < UID.len(); i++) {
				FIFOBlob.writen(UID[i], 'b');
			}
			directCommand(0xC2);    // Clear FIFO / status registers
			regWrite(0x1D, FIFOBlob.len() >> 8);    // Write # of Transmitted Bytes (MSB)
			regWrite(0x1E, FIFOBlob.len() << 3);    // Write # of Transmitted Bytes (LSB)
			FIFOWrite(FIFOBlob);
			setBit(0x09, 7);        // Make sure no_CRC_rx is set
			setBit(0x05, 0);        // Set antcl
			directCommand(0xC5);    // Transmit contents of FIFO without CRC
			server.log("Transmitting anticollision packet:");
		}
		
		server.log("FIFO length: " + FIFOBlob.len());
//		foreach (i, byte in FIFOBlob) {
//			server.log(format("Byte %i: 0x%02X", i, byte));
//		}
		
	}

	// Receive Handler
	function receiveHandler() {
		local FIFO = FIFORead();
//	    foreach (byte in FIFO) {
//			server.log(format("Byte: 0x%02X", byte));
//		}
		if (receiveType == RECEIVE_TYPE_ATQA) {
			if (!(atqa & 0xF020) && FIFO.len() == 2) {
        server.log("SET CARDPRESENT = 1");
        cardPresent = 1;
				atqa = FIFO[1] << 8 | FIFO[0];
				server.log(format("Valid ATQA Received: %04X", atqa));
				imp.sleep(0.005);   // Wait 5ms for card to be ready (maybe)
				cascadeLevel = 1;
				anticollision(0x20, false, false);
			}
			else {
        server.log("SET CARDPRESENT = 0");
        cardPresent = 0;
				server.log("Invalid ATQA!");
			}
		}
		else if (receiveType == RECEIVE_TYPE_ANTICOLLISION) {
			server.log("Received anticollision frame.");
      foreach (byte in FIFO) {
  		  server.log(format("Byte: 0x%02X", byte));
		  }
			local final = false;
			if (FIFO[0] != 0x88) {  // Check for cascade tag
				UID.writen(FIFO[0], 'b');
				final = true;
			}
			UID.writen(FIFO[1], 'b');
			UID.writen(FIFO[2], 'b');
			UID.writen(FIFO[3], 'b');
			local BCC = FIFO[0] ^ FIFO[1] ^ FIFO[2] ^ FIFO[3];
			if(BCC == FIFO[4]){
				server.log("valid BCC!");
				//send next anticollision frame to get the rest of the UID
				anticollision(0x70, true, final);
			}else{
				server.log("invalid BCC!");
				return;
			}
		}
		else if (receiveType == RECEIVE_TYPE_SAK) {
			server.log("Received SAK");
			if (FIFO[0] & 0x04) {
				server.log("INCOMPLETE UID");
				cascadeLevel++;
				anticollision(0x20, false, false);
			}
			else if (FIFO[0] & 0x20) {
				server.log("COMPLETED UID!");
        hardware.pin9.write(0);
				foreach (i, byte in UID) {
					server.log(format("UID Byte %i: 0x%02X", i, byte));
				}
        UID.flush();
        UID.seek(0);
        // receiveType = 0;
        directCommand(0xC2);    // Clear FIFO / status registers
        // cardPresent = 0;
        // enterWakeup();
			}
		}
		else {
			server.log("Receive type [" + receiveType + "] unknown!");
		}
	}

	// Interrupt Handler
	function interruptHandler() {
		if (irq.read()) {
			// Clear previous interrupt flags
			collision = 0;
			server.log("***BEGIN INTERRUPT LIST***");
			// Read main interrupt register first
			local mainInterruptReg = regRead(0x17);
			// Is there a smarter way to do this?
			if (mainInterruptReg & 0x80) {
				// Oscillator frequency has stabilized
				server.log("Oscillator frequency stable.");
			}
			if (mainInterruptReg & 0x40) {
				// FIFO water level (full or empty)
				server.log("FIFO water level!")
				//local fifo_contents = FIFORead();
			}
			if (mainInterruptReg & 0x20) {
				// Start of receive
				server.log("Receive start.")
			}
			if (mainInterruptReg & 0x10) {
				// End of receive
				server.log("Receive end.")
				imp.wakeup(0.01, receiveHandler.bindenv(this));
			}
			if (mainInterruptReg & 0x08) {
				// End of transmit
				server.log("Transmit end.");
			}
			if (mainInterruptReg & 0x04) {
				// Bit collision
				server.error("Bit collision!");
				regPrint(0x1C); // Print Collision Display Register
				collision = 1;
			}
			if (mainInterruptReg & 0x02) {
				// Timer or NFC interrupt
				local timerInterruptReg = regRead(0x18);
				if (timerInterruptReg & 0x80) {
					// Termination of direct command
					server.log("Direct command complete.");
				}
				if (timerInterruptReg & 0x40) {
					// No-response timer expire
					server.log("No-reponse timer expired");
          // enterWakeup();
				}
				if (timerInterruptReg & 0x20) {
					// General purpose timer expire
					server.log("General purpose timer expired");
				}
				if (timerInterruptReg & 0x10) {
					// NFC: External field greater than Target activation level
					server.log("NFC: External field > target activation level");
				}
				if (timerInterruptReg & 0x08) {
					// NFC: External field less than Target activation level
					server.log("NFC: External field dropped below target activation level");
				}
				if (timerInterruptReg & 0x04) {
					// NFC: Collision detected during RF Collision Avoidance
					server.log("NFC: Collision detected");
				}
				if (timerInterruptReg & 0x02) {
					// NFC: Minimum guard time expire
					server.log("NFC: Minimum guard time expired");
				}
				if (timerInterruptReg & 0x01) {
					// NFC: Initiator bit rate recognized
					server.log("NFC: Initiator bit rate recognized");
				}
			}
			if (mainInterruptReg & 0x01) {
				// Error or Wake-up interrupt
				local wakeInterruptReg = regRead(0x19);
				if (wakeInterruptReg & 0x80) {
					// CRC error
					server.error("CRC error!");
				}
				if (wakeInterruptReg & 0x40) {
					// Parity error
					server.error("Parity error!");
				}
				if (wakeInterruptReg & 0x20) {
					// Soft framing error
					server.error("Soft framing error!");
				}
				if (wakeInterruptReg & 0x10) {
					// Hard framing error
					server.error("Hard framing error!");
				}
				if (wakeInterruptReg & 0x08) {
					// Wake-up interrupt
					server.log("Wake-up interrupt");
				}
				if (wakeInterruptReg & 0x04) {
					// Wake-up interrupt due to Amplitude Measurement
					server.log("Wake-up interrupt due to Amplitude Measurement");
				}
				if (wakeInterruptReg & 0x02) {
					// Wake-up interrupt due to Phase Measurement
					server.log("Wake-up interrupt due to Phase Measurement");
				}
				if (wakeInterruptReg & 0x01) {
					// Wake-up interrupt due to Capacitance Measurement
					server.log("Wake-up interrupt due to Capacitance Measurement");
					server.log(format("ADC value: 0x%02X", regRead(0x20)));
          server.log("cardPresent: " + cardPresent);
					
					hardware.pin9.write(0);
					imp.wakeup(0.1, function(){hardware.pin9.write(1)});
					if(cardPresent==0){
            server.log("TURNED CAPSENSE CARPRESENT = 1");
            cardPresent=1;
            enterReady();
          }
				}            
			}
		}
		else {
			server.log("****END INTERRUPT LIST****");
		}
	}

	// Calibrate capacitive sensor
	function calibrateCapSense() {
		server.log("Calibrating cap sensor...");
		regWrite(0x2E, 0x01);   // Enable automatic calibration, gain 6.5V/pF
		directCommand(0xDD);    // Calibrate
		regWrite(0x3A, 0xA9);   //Set delta and auto-avg settings
		local capSenseDisplayReg = regRead(0x2F);
		if (capSenseDisplayReg & 0x04) {
			local cal = capSenseDisplayReg >> 3 & 0x1F;
			server.log("Capacitive sensor calibrated. Value: " + cal);
		}
		else if (capSenseDisplayReg & 0x02) {
			server.log("Calibration error!");
		}
	}

	// Measure the capacitive sensor once
	function measureCapSense() {
		directCommand(0xDE);
		local capResult = regRead(0x20);
		server.log(capResult);
		//imp.wakeup(0.5, measureCapSense);
	}

	// Send REQA
	function sendREQA() {
		server.log("Sending REQA");
		receiveType = RECEIVE_TYPE_ATQA;    // Expect ATQA in response
		directCommand(0xC2);    // Clear (not necessary, but what the hell)
		directCommand(0xC6);    // Send REQA
//		if (cardPresent) imp.wakeup(1, sendREQA.bindenv(this));
	}
  
  // Turn Radios off (RX and TX)
  function radioOFF(){
    clearBit(0x02, 6); 
    clearBit(0x02, 3);
    server.log("Radios off."); 
  }

	function enterReady() {
		// cardPresent = 1;
		regWrite(0x02, 0x80);   // Operation Control - Enable ready mode (en -> 1)
		regWrite(0x05, 0x01);   // Anticollision bit set (antcl)
		server.log("Ready mode enabled.");
		// Should technically wait for oscillator to stabilize here - but how?
		
		directCommand(0xD6);    // Adjust Regulators
		directCommand(0xD8);    // Calibrate Antenna
		regWrite(0x02, 0xC8);   // Tx/Rx Enable
		imp.sleep(0.005);       // Wait 5ms for reader field to stabilize
		server.error("INSERT CARD NOW!");
		hardware.pin9.write(0);
		imp.sleep(1);
		hardware.pin9.write(1);
		
		sendREQA();             // Send REQA
		
		// imp.wakeup(10, function(){ this.cardPresent <- 0; enterWakeup(); });
	}

	function enterWakeup(){
    server.log("Entering Wakeup");
		regWrite(0x31, 0x01); //Wake-up timer Control - CapSense at every timeo0ut (wcap -> 1)
		regWrite(0x02, 0x04); //Operation Control - Enable wakeup mode (wu -> 1)
	}
  
  // Check to make sure the IRQ pin isn't stuck high
	function pollIRQ() {
		if (irq.read()) {
      interruptHandler();
		}else{ 
      // server.log("hi");
      // server.log("cardPresent: " + cardPresent);
      if(sleepTime < time() && cardPresent==1){
        server.log("TURNED RESET CARDPRESENT=0")
        cardPresent = 0;
        enterWakeup();
        server.log("cardPresent: " + cardPresent);
		  }
		}
	}
}

//irq <- hardware.pin1;
//spi <- hardware.spi257;
//cs_l <- hardware.pin8;
RFID <- AS3911(hardware.pin1, hardware.spi257, hardware.pin8);

// Configure I/O
RFID.irq.configure(DIGITAL_IN_WAKEUP, RFID.interruptHandler.bindenv(RFID));
RFID.cs_l.configure(DIGITAL_OUT);
RFID.cs_l.write(1);
// SPI Mode 1: CPOL = 0, CPHA = 1
RFID.spi.configure(CLOCK_IDLE_LOW | CLOCK_2ND_EDGE, FREQ);

imp.configure("RFID AS3911", [], []);

//LED
hardware.pin9.configure(DIGITAL_OUT_OD);
hardware.pin9.write(1);

// Initialization
RFID.directCommand(0xC1);    // (C1) Set Default
RFID.directCommand(0xC2);    // (C2) Clear
RFID.regWrite(0x00, 0x0F);   // IO Config 1 - Disable MCU_CLK output
RFID.regWrite(0x01, 0x00);   // IO Config 2 - Defaults
RFID.regWrite(0x02, 0x00);   // Operation Control - Defaults, power-down mode
RFID.regWrite(0x03, 0x08);   // Mode Definition - ISO14443a (no auto rf collision)
RFID.regWrite(0x04, 0x00);   // Bit Rate Definition - fc/128 (~106kbit/s) lowest
RFID.regWrite(0x0E, 0x04);   // Mask Receive Timer - 4 steps, ~19us (minimum)
RFID.regWrite(0x0F, 0x00);   // No-response Timer - 21 steps, ~100us (MSB)
RFID.regWrite(0x10, 0x15);   // No-response Timer - 21 steps, ~100us (LSB)

RFID.directCommand(0xCC);    // Analog Preset
RFID.calibrateCapSense();    // Calibrate Capacitive Sensor
RFID.regWrite(0x2A, 0x00);   // Regulator Voltage Control - Automatic, Defaults


function poll(){
  RFID.pollIRQ();
  imp.wakeup(1, poll);
}

RFID.enterWakeup();
// RFID.enterReady();
poll();
// imp.wakeup(6, function(){ RFID.radioOFF(); RFID.cardPresent=0; hardware.pin9.write(1);});


server.log("End of code.");

