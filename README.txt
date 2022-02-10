SIGNAL_CMDR is a Railway Singal Manager for Model Railroading intended for an ARDUINO  MEGA 2560

Version 1.4 is an upgrade to a previous version running in the Belleville Model Railroad Club where
 several Blocks were applied to manage a group of Signals. Plug it into a MEGA, Download and open the Serial Monitor (115200).
  A Default Configuration is created and you can TEST it with no Sensors or Signals attached. The Verbose Option prints out 
  every significant Event from the Detector Level op. The Built in SIM functions allow testing of the Configuration, and also
  testing of Signal Aspects when you have Signals connected.

The application has 4 fundamental Object Structures:
   SENSORS   -Individual Sensors (12 Max) coming to an Analog Input. These use UV,or Photo, Sensors with a 
              Resistor Ladder to detect Beam Interruption. Each maintains Calibration Values to convert analog values
              to Digital - Train vs No Train. A Deadband is applied (Dflt = 750 ms) to the Falling Edge to prevent aliasing 
              where the status rapidly toggles as the sensors detect gaps in the passing train.

   DETECTORS -a Pair of member SENSORS, Left and Right, physically separated by 3 - 5 cm. The member Sensors are polled rapidly
              (about 4 ms) to detect which side triggered first to learn Train Direction. A Compass Orientation
              is associated with the member Sensors (Left vs Right) to translate Train movements into W,E,N, or S. 
              Max 6 DETECTORS, each with a MilePost associated with it. 

   BLOCK     -Up to 4 Named Track Segments that are Terminated at each end (Left and Right) by a Detector. A Compass
               Orientation is associated with the Ends to translate Train Movements into realistic Train Directions. 
               Block Logic sets Entry Permissive Levels (0 - 3) for each end, and sets Entry Signal Aspects to be applied.
               There are NO Exit Signals to a Block - Signals facing the other direction are the ENTRY to the NEXT Block.
               A Block can also have up to 2 Turnouts within and each has a Facing Points Direction. A Turnout is mapped 
               to a Digital Input. Turnouts modify Permissive Levels.
               Blocks have a Track Type setting - MAINLINE (Dflt), and SIDING that apply to permissive Levels a so Signals
               Blocks have an Adjacent Block ID. Adjacent Block Entry Permissives are referenced for Entry Permissive and
               Signal Selection.
                 Becasue this is for model Railroading there is always the possibility that someone can lift a train off the 
               track, so an OCC Timeout is applied (Dflt = 120 sec) that will Clear the OCC Status if not activity is detected.
              OPEN ENDED BLOCKs (Detector  1 End Only) are supported for SPURs and the outer edge of Signaled Territory. These
               go OCCUPIED when a Train is Detected and Timeout (15 sec) to return to the UNOCC state.

   SIGNALS    Associated with one end of a BLOCK and display the Entry Signal Aspect produced by that Block. Each Signal (Max = 16)
               can have 1,2, or 3 Lamps, and each Signal can be one of 3 Types:
                  1) R,G,Y LED with 3 Digital Outputs, one per colour, for each Lamp - separate YLW LED
                  2) R,G   LED - 2 Outputs per Lamp. YLW is created with Both outputs = ON
                  3) R,G  Bipolar LED - 2 Outputs per Lamp. YLW is created by reversing active output every ms
               All LEDs MUST have proper Resistors in series with each output (220 to 1000 ohm)

This App has Robust Debug and Monitoring capabilities through the Serial Monitor portal. The Vocabuly simplifies command interpretation
 and provide ample embedded HELP to make your application go easily. Embedded SIM Features allow Testing and Demonstration of your 
 Configuration. The HELP includes Command Specific Help (example >HELP ADD )to explain the steps of Configuration for your specific
 application. A very intuitive approach to Configure makes SIGNAL_CMDR an ideal application for railway signalling.
 After you complete the Configuration and CAL steps, SAVE it to EEPROM to remain correctly configured for your application.

This makes use of the millis() function to provide Clock Time and an array of millisecond soft timers that make the program very efficient
 All tasks in the main loop are executed 1 per pass and within the  newms  condition so that background IO functions proceed and so
 timing overruns are avoided. I will never use a delay() function, as this is a blocking task that negatively affects timing for
 everything else. So the use of State machines and soft timers makes this an elegant and easy to understand application.

There are 9 modules (files) plus the SIGNAL.H file so any programmer can review and understand how it functions easily
Support Requests or suggestions for this application can be sent to  RossCameron.ca@gmail.com
 ENJOY !