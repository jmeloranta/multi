WSJT-X aggregator (Linux only)

Background

My Elecraft K4D has two receivers, which can be used in diversity receive with my TX (inverted L) and RX antennas (beverages) on low bands. So the first RX antenna is connected to inverted L and second to beverage. The radio outputs audio from the RXs to left and right stereo channels. But how to take advantage of this set up on FT8? The first thought was to combine the audio channels (mono) and feed that into WSJT-X. Even though the radio has the two RXs phase locked, there appears to be some differences in the phase factor of their audio signal. Hence it is not a a good idea to combined them directly. Reading them separately with WSJT-X is not possible since it only supports one audio input. So, what to do? The proper solution would to have WSJT-X GUI read two audio inputs but that's easier said than done. One could run two WSJT-X's separately for the RXs but running QSOs is clumsy.

Solution (FT8)

This program (multi) sits between two WSJT-X GUI processes and their JT9 decoders. One of the GUIs (master) acts as the main program and all QSOs etc. go through it. It is set up to read audio from the left stereo channel. It will feed that audio data through shared memory to JT9 for decoding. The second WSJT-X (right audio channel) just will provide the audio data to JT9 and show the corresponding waterfall display (no decodes). Multi sits between the two WSJT-X GUIs and two separate JT9 decoder processes. It collects the decodes, chooses the decodes with the best signals, inserts signal source & strength information into it, and finally passes them to the master WSJT-X GUI. Since the JT9 decoder path is hardcoded into WSJT-X GUI, multi program is placed as jt9 and the actual original decoder is renamed as jt9.x . Note that multi has also option for collecting statistics of the decodes that can be useful for antenna comparisons (within the transmission period of FT8) since the decodes from both antennas are performed simultaneously. Multi.c has compile time option for this (see the source code). Although I have not tried multi likely also works with FT4, JT65, Q65, etc. since the JT9 program supports these modes.

Installation

Installation assumes that the regular WSJT-X is installed under /usr/bin and the modified WSJT-X will be under /usr/local/bin. install-multi script copies the required files from /usr/bin to /usr/local/bin, renames /usr/local/bin/jt9 to /usr/local/bin/jt9.x and multi program becomes /usr/local/bin/jt9 . To run the program start two separate copies (from /usr/local/bin) of WSJT-X. First without any arguments and the second with "-r 2" (2 refers to WSJT-X configuration name). Provided script start-multi shows how to do this step. Note that the radio names are also used to name the shared memory segments through which the audio data is provided. These are hardcoded into multi program (see the code). The main window of the second WSJT-X GUI can be minimized (but do not close it) as it will provide no decode output. Its waterfall display is useful because it shows the signals present on the slave. The first WSJT-X GUI instance will show all decodes. Make sure that your radio is set up correctly; audio from the two RXs are on the left & right stereo channels and the radio is in diversity receive mode. Additionally, rig control must be disabled in the slave WSJT-X - otherwise the two WSJT-X instances will both try to communicate with the radio. If you are using WSJT-X Improved, decode filtering does NOT work. Filters must be disabled - otherwise most decodes won't show. No idea why this is happening but there is now built-in filtering. Make sure to specify identical decode parameters in both instances of WSJT-X GUI as JT9 gets them from the corresponding shared memory segments. This would be very important if performing antenna comparisons (avoid differences in decoding algorithms). Review environment variable settings in start-multi script.

Operation

Using multi is more or less transparent. The master WSJT-X GUI automatically gets the best decodes and there are no changes how the QSOs are run. The only new information that appears on the decodes is the source RX and indication how much stronger that RX was compared to the other one. This information appears where '~' appears in the normal decode output. The status info is as follows:

=  Signal received by RX a and b are equally strong (within 2 dB)
a  Signal received by RX a is at least 2 dB stronger than b
b  Signal received by RX b is at least 2 dB stronger than a
A  Signal received by RX a is more than 8 dB stronger than b
B  Signal received by RX b is more than 8 dB stronger than a
A! Only RX a received the signal (no signal on b)
B! Only RX b received the signal (no signal on a)

Observations

This approach improved my low band decodes a lot. Just like regular diversity receive, it works best with two different types of receive antennas (arrival angle & polarization). It does require a fast computer - otherwise there will be missed decodes. These missed decodes appear as A! or B!, which can be mistakenly though of either of the RX not receiving the signal at all. The multithreaded decoder seems to be a bit unreliable, so keep that off in both WSJT-X instances (especially when comparing antennas). Multi threaded decoder seems to behave somewhat erratic - so disable it for both WSJT-X instances. There are still some strange things going on with JT9: under some circumstances it gives different decodes although given the same input data. Although some of these issues are related to using <...> where one JT9 process uses them but the other not.

PROBLEM

There was earlier but I had accidentally "decode after EME delay" set (in one JT9 instance only).

