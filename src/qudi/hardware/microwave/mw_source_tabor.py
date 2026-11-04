# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware file to control tabor(new microwave device in labor) microwave device.
In our lab, Tabor only works with usb, not visa

edited by PI3 of University Stuttgart

"""

# try:
#     import pyvisa as visa
# except ImportError:
#     import visa
import pywinusb.hid as hid
import time
import numpy as np

from qudi.util.mutex import Mutex
from qudi.core.configoption import ConfigOption
from qudi.interface.microwave_interface import MicrowaveInterface, MicrowaveConstraints
from qudi.util.enums import SamplingOutputMode

class MicrowaveTabor(MicrowaveInterface):
    """  Hardware file for Tabor. 

    Example config for copy-paste:

    tabor_12G:
        module.Class: 'microwave.mw_source_tabor.MicrowaveTabor'
        options:
            channel : 1                 
            teVendorId  : 0x168C
            teLucidDesktopId  : 0x6002  # Use this for Lucid Desktop - 6GHz
            teLucidPortableId : 0x6081  # Use this for Lucid Portable - 6GHz + 1 Channel
            teLucidBenchtopId : 0x1202  # Use this for Lucid Benchtop - 12GHz + 2 Channels
            BUFFER_SIZE : 256
            FM_MODE     : 0             # 1-ON, 0-OFF          

    tabor_3G:
        module.Class: 'microwave.mw_source_tabor.MicrowaveTabor'
        options:
            channel : 1
            teVendorId  : 0x168C
            teLucidDesktopId  : 0x6002  # Use this for Lucid Desktop - 6GHz
            teLucidPortableId : 0x6081  # Use this for Lucid Portable - 6GHz + 1 Channel
            teLucidBenchtopId : 0x3002  # Use this for Lucid Benchtop - 12GHz + 2 Channels
            BUFFER_SIZE : 256
            FM_MODE     : 0             # 1-ON, 0-OFF
    """

    teVendorId  = ConfigOption('teVendorId', default = 0x168C)
    teLucidDesktopId  = ConfigOption('teLucidDesktopId', default = 0x6002)
    teLucidPortableId = ConfigOption('teLucidPortableId', default = 0x6081)
    teLucidBenchtopId = ConfigOption('teLucidBenchtopId', default = 0x3002)
    channel = ConfigOption('channel', default= 1)
    BUFFER_SIZE = ConfigOption('BUFFER_SIZE', default = 256)
    _rising_edge_trigger = ConfigOption('rising_edge_trigger', default=True)
    _advance_mode = ConfigOption('advance_mode', default='AUTO')
    _config_freq_min = ConfigOption('frequency_min', default=None)
    _config_freq_max = ConfigOption('frequency_max', default=None)
    _config_power_min = ConfigOption('power_min', default=None)
    _config_power_max = ConfigOption('power_max', default=None)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = Mutex()
        self._rm = None
        self._device = None
        self._model = 'LS3082B'
        self._constraints = None
        self._cw_power = -20
        self._cw_frequency = 2.0e9
        self._scan_power = -10
        self._scan_frequencies = None
        self._scan_mode = None
        self._scan_sample_rate = 0.
        self._temp_read = None # temp variable, use to store the readed information
                               # used in func readData
        self._list_state = 0   #tabor LS3082B is broken to show the real state of list
    def on_activate(self):
        """ Initialisation performed during activation of the module."""
        # Connect to hardware
        self._device = hid.HidDeviceFilter(
            vendor_id = self.teVendorId, 
            product_id = self.teLucidPortableId
            ).get_devices()[0]
        
        # Open device
        # self._device.open() not necessary in usb mode

        # Reset device
        self._command_wait('*CLS\n')
        self._command_wait('*RST\n') # reset the whole Tabor
        self._command_wait('*IDN?\n')
        # set the default states
        command_channel = ':INST ' + str(self.channel)
        self._command_wait(command_channel) # choose channel
        self._command_wait(':INIT:GATE OFF')
        self._command_wait(':TRIG:SOUR EXTernal')
        self._command_wait(':TRIG:EDG POS')
        self._command_wait(':TRIG:ADV STEP')  # only running with trigger
        
        # Generate constraints
        # self._model == 'LS3082B':
        # Lucid family limits. The original (1e8, 3e9) was a workaround; the
        # hardware floor is 9 kHz and the ceiling depends on the model, so set
        # the per-model values from config and let these act as fallbacks.
        freq_limits = (9e3, 12e9)
        power_limits = (-100, 20)

        # Apply configured soft-boundaries
        if self._config_power_min is not None:
            assert self._config_power_min < power_limits[1]
            power_limits = (max(power_limits[0], self._config_power_min), power_limits[1])
        if self._config_power_max is not None:
            assert self._config_power_max > power_limits[0]
            power_limits = (power_limits[0], min(power_limits[1], self._config_power_max))
        # Config values define the range directly. The original code used
        # max()/min() which meant a configured minimum below the hardcoded
        # floor could never take effect.
        if self._config_freq_min is not None:
            freq_limits = (float(self._config_freq_min), freq_limits[1])
        if self._config_freq_max is not None:
            freq_limits = (freq_limits[0], float(self._config_freq_max))
        assert freq_limits[0] < freq_limits[1], \
            'frequency_min must be below frequency_max'
        self._constraints = MicrowaveConstraints(
            power_limits=power_limits,
            frequency_limits=freq_limits,           
            scan_size_limits=(2, 4096),
            # Upper bound is the hardware dwell floor of 100 us, i.e. 10 kHz.
            # The Lucid manual gives the same 100 us floor for :LIST:DEF and
            # :FRSW:TIME. The original cap of 100 Hz throttled the instrument
            # by two orders of magnitude.
            sample_rate_limits=(0.1, 10000),
            scan_modes=(SamplingOutputMode.JUMP_LIST, SamplingOutputMode.EQUIDISTANT_SWEEP)
        )

        self._scan_frequencies = self._constraints.min_frequency
        self._scan_power = self._constraints.min_power
        self._cw_power = self._constraints.min_power
        self._cw_frequency = 2870.0e6
        self._scan_mode = SamplingOutputMode.JUMP_LIST
        self._scan_sample_rate = self._constraints.max_sample_rate

    def on_deactivate(self):
        """ Cleanup performed during deactivation of the module. """
        # Close device
        self._device.close()
    
    @property
    def constraints(self):
        return self._constraints   
    
    @property
    def is_scanning(self):
        """Read-Only boolean flag indicating if a scan is running at the moment. Can be used together with
        module_state() to determine if the currently running microwave output is a scan or CW.
        Should return False if module_state() is 'idle'.

        @return bool: Flag indicating if a scan is running (True) or not (False)
        """
        with self._thread_lock:
            return (self.module_state() != 'idle') and not self._in_cw_mode()
        
    @property
    def cw_power(self):
        """The CW microwave power in dBm. Must implement setter as well.

        @return float: The currently set CW microwave power in dBm.
        """
        with self._thread_lock:
            return self._cw_power

    @property
    def cw_frequency(self):
        """The CW microwave frequency in Hz. Must implement setter as well.

        @return float: The currently set CW microwave frequency in Hz.
        """
        with self._thread_lock:
            return self._cw_frequency

    @property
    def scan_power(self):
        """The microwave power in dBm used for scanning. Must implement setter as well.

        @return float: The currently set scanning microwave power in dBm
        """
        with self._thread_lock:
            return self._scan_power

    @property
    def scan_frequencies(self):
        """The microwave frequencies used for scanning. Must implement setter as well.

        In case of scan_mode == SamplingOutputMode.JUMP_LIST, this will be a 1D numpy array.
        In case of scan_mode == SamplingOutputMode.EQUIDISTANT_SWEEP, this will be a tuple
        containing 3 values (freq_begin, freq_end, number_of_samples).
        If no frequency scan has been specified, return None.

        @return float[]: The currently set scanning frequencies. None if not set.
        """
        with self._thread_lock:
            return self._scan_frequencies

    @property
    def scan_mode(self):
        """Scan mode Enum. Must implement setter as well.

        @return SamplingOutputMode: The currently set scan mode Enum
        """
        with self._thread_lock:
            return self._scan_mode

    @property
    def scan_sample_rate(self):
        """Read-only property returning the currently configured scan sample rate in Hz.

        @return float: The currently set scan sample rate in Hz
        """
        with self._thread_lock:
            return self._scan_sample_rate

# --------------------------------------------------------------------------------------------
        
    def set_cw(self, frequency, power):
        """Configure the CW microwave output. Does not start physical signal output, see also
        "cw_on".

        @param float frequency: frequency to set in Hz
        @param float power: power to set in dBm
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to set CW parameters. Microwave output active.')
            self._assert_cw_parameters_args(frequency, power)

            if not self._in_cw_mode():
                self._command_wait(':LIST OFF')
                self._list_state = 0
                self._command_wait(':FRSW OFF')
                
            self._command_wait(':INIT:CONT ON')
            self._command_wait('FREQuency {}\n'.format(frequency))
            self._command_wait('POWer {}\n'.format(power))
            self._cw_frequency = float(self._command_wait('*FREQuency?\n'))
            self._cw_power = float(self._command_wait( '*POWer?\n'))
  

    def configure_scan(self, power, frequencies, mode, sample_rate):
        """
        """
        # Accept the mode as a string as well as the enum. Remote clients
        # (the qudi console over rpyc) cannot pass the driver's own enum
        # object, so a name is the only practical way to select a mode there.
        if isinstance(mode, str):
            try:
                mode = SamplingOutputMode[mode.upper()]
            except KeyError:
                raise ValueError(
                    'Unknown scan mode {!r}. Valid names: {}'.format(
                        mode, ', '.join(m.name for m in SamplingOutputMode))
                )
        with self._thread_lock:
            # Sanity checks
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to configure frequency scan. Microwave output active.')
            self._assert_scan_configuration_args(power, frequencies, mode, sample_rate)

            # configure scan according to scan mode
            self._scan_mode = mode
            self._scan_sample_rate = sample_rate
            self._scan_power = power
            if mode == SamplingOutputMode.JUMP_LIST:
                self._scan_frequencies = np.asarray(frequencies, dtype=np.float64)
                # self._write_list()
                # self._write_sweep()
            elif mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                self._scan_frequencies = tuple(frequencies)
                self._write_sweep()

            self._set_trigger_edge() 

    def off(self):
        """Switches off any microwave output (both scan and CW).
        Must return AFTER the device has actually stopped.
        """
        with self._thread_lock:
            # Always send the command. The module state can disagree with the
            # hardware (it is only updated by this class, and list/sweep state
            # queries are unreliable on this firmware), so trusting
            # module_state() here left the RF running on a no-op.
            self._command_wait(':OUTP OFF')
            if self._list_state:
                self._command_wait(':LIST OFF')
                self._list_state = 0
            self._command_wait(':FRSW OFF')
            time.sleep(0.3)
            self._wait_output_state(expected=False)
            if self.module_state() != 'idle':
                self.module_state.unlock()

    def cw_on(self):
        """ Switches on cw microwave output.

        Must return AFTER the output is actually active.
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                if self._in_cw_mode():
                    return
                raise RuntimeError(
                    'Unable to start CW microwave output. Microwave output is currently active.'
                )

            if not self._in_cw_mode():
                self._command_wait(':LIST OFF')
                self._list_state = 0
                self._command_wait(':FRSW OFF')
                self._command_wait('FREQuency {}\n'.format(self._cw_frequency))
                self._command_wait('POWer {}\n'.format(self._cw_power))

            self._command_wait(':OUTP ON')
            time.sleep(0.3)
            self._wait_output_state(expected=True)
            self.module_state.lock()

    def start_scan(self):
        """Switches on the microwave scanning.

        Must return AFTER the output is actually active (and can receive triggers for example).
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                if not self._in_cw_mode():
                    return
                raise RuntimeError('Unable to start frequency scan. CW microwave output is active.')
            assert self._scan_frequencies is not None, \
                'No scan_frequencies set. Unable to start scan.'

            # LIST mode
            if self._scan_mode == SamplingOutputMode.JUMP_LIST:
                if not self._in_list_mode():
                    self._write_list()
                                   
                self._command_wait(':OUTP ON')
                self._wait_output_state(expected=True)
            # SWEEP mode
            elif self._scan_mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                if not self._in_sweep_mode():
                    self._write_sweep()

                self._command_wait(':OUTP ON')
                self._wait_output_state(expected=True)
            else:
                raise RuntimeError(
                    f'Invalid scan mode encountered ({self._scan_mode}). Please set scan_mode '
                    f'property before configuring or starting a frequency scan.'
                )
            self.module_state.lock()

    def reset_scan(self):
        """Reset currently running scan and return to start frequency.
        Does not need to stop and restart the microwave output if the device allows soft scan reset.
        """

        """
        ATTENTION: Tabor seems really stupid, during each reset we much set the parameters agagin.
        """
        with self._thread_lock:
            if self.module_state() == 'idle':
                return
            if self._in_cw_mode():
                raise RuntimeError('Can not reset frequency scan. CW microwave output active.')
            """ the command above can lead to big bug.
            Because tabor cannot correctly output thereal state of list mode"""

            if self._scan_mode == SamplingOutputMode.JUMP_LIST:
                # self._command_wait(':OUTP OFF')
                # self._write_sweep()
                # self._command_wait(':OUTP ON')
                temp = 0
            elif self._scan_mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                self._command_wait(':OUTP OFF')
                self._command_wait(':OUTP ON')


    def readData(self, data):
        # remove '\n'
        strData = ''.join([str(elem) for i, elem in enumerate(data)])
        text = ''
        if len(data) == 0:
            print("Data is empty")
        elif strData: #and not strData.isspace() and strData.isprintable():
            for c in data:
                if c != 0:
                    text+= chr(c)
            # print (text)
            self._temp_read = text
        return None

    def _command_wait(self, command_str):
        """ Writes the command in command_str via Pywinusb and waits until the device has finished
        processing it.

        @param str command_str: The command to be written
        """
        if not self._device:
            print("No device provided")
            return

        self._temp_read = None
        buffer = [0x00] * self.BUFFER_SIZE
        sendData = bytearray(command_str, 'utf-8')
        for i in range(len(sendData)):
            buffer[i + 3] = sendData[i]

        self._device.open()
        self._device.set_raw_data_handler(self.readData)
        self._device.send_output_report(buffer)

        for _ in range(75):
            if self._temp_read is not None:
                break
            time.sleep(0.02)
        self._device.close()

        response = self._temp_read
        if response is not None and 'SCPI ERROR' in response.upper():
            raise RuntimeError(
                'Instrument rejected command {!r}: {}'.format(
                    command_str.strip(), response.strip())
            )
        return response
    
    
    def _wait_output_state(self, expected, attempts=10, interval=0.2):
        """Wait for the output relay to reach `expected`, but never block forever.

        The firmware will not always answer ':OUTP?' (and does not report
        errors over HID), so the original unbounded while-loops could spin
        indefinitely while holding the module thread lock. This gives up after
        a bounded number of reads and logs instead of hanging.
        """
        for _ in range(attempts):
            time.sleep(interval)
            state = self._command_wait(':OUTP?')
            if state is None:
                continue
            if (state.strip() == 'ON') == expected:
                return True
        self.log.warning(
            'Output state did not read back as {} after {} attempts. '
            'Proceeding anyway.'.format('ON' if expected else 'OFF', attempts)
        )
        return False

    def _in_list_mode(self):
        return self._list_state == 1

    def _in_sweep_mode(self):
        return self._command_wait(":FRSW?") == 'ON'

    def _in_cw_mode(self):
        return not (self._in_list_mode() or self._in_sweep_mode())

    def _write_list(self):
        """Download a frequency list to the instrument.

        Syntax verified on an LS6081P (firmware 07CCE000), not taken from the
        Lucid Programming Manual Rev. 1.1, which is wrong in three ways:

          * It shows spaces after the commas in :LIST:DEF. The firmware
            rejects that with "Invalid point"; the compact form is required.
          * It documents <step> as 2..4096 but its own example uses step 1.
            The firmware enforces the range: step 1 yields "Invalid point".
          * It documents :LIST? as returning 1/0 by state. It always returns
            OFF on this firmware, so list state is tracked in software.

        <advance> 0 = auto-advance, 1 = triggered. <dwell> is in SECONDS.
        """
        if not self._in_list_mode():
            self._command_wait(':FRSW OFF')
            self._command_wait(':LIST ON')
            self._list_state = 1
            time.sleep(0.3)

        dwell = 1.0 / float(self._scan_sample_rate) if self._scan_sample_rate else 1e-3
        if dwell < 100e-6:
            dwell = 100e-6

        self._command_wait(':POWer {}'.format(self._scan_power))

        triggered = str(self._advance_mode).upper().startswith('TRIG')
        advance = 1 if triggered else 0

        # Step 1 is implicit and cannot be defined on this firmware, so the
        # first frequency goes on step 2.
        count = len(self._scan_frequencies)
        for index in range(count):
            last = 1 if index == count - 1 else 0
            self._command_wait(
                ':LIST:DEF {},{},{},{},{},{}'.format(
                    index + 2,
                    int(self._scan_frequencies[index]),
                    self._scan_power,
                    last,
                    advance,
                    dwell
                )
            )

        if triggered:
            # Each step waits for an external trigger on the rear TRIG input.
            self._set_trigger_edge()
            self._command_wait(':TRIG:SOUR EXTernal')
            self._command_wait(':INIT:CONT OFF')
            self._command_wait(':TRIG:ADV STEP')
        else:
            # Auto-advance needs continuous initiation; with :INIT:CONT OFF
            # the unit sits on the first entry waiting for a trigger that
            # never comes.
            self._command_wait(':INIT:CONT ON')

    def _write_sweep(self):
        """Configure and arm a frequency sweep.

        Bench-verified on an LS6081P (firmware 07CCE000). Sweep mode requires
        TRIGGERED run mode with the timer as trigger source, which is the
        opposite of list mode: in continuous run mode the sweep accepts every
        command, reports ':FRSW?' as ON, and never moves.

        The engine interpolates into discrete points and steps between them;
        there is no analog ramp. Use a fine :FRSW:STEP with a short
        :FRSW:TIME for a sweep that looks continuous.

        :TRIG:ADV ONC is required for one trigger to run a whole sweep.
        ':TRIG:ADV ONCE' and ':TRIG:ADV 1' are accepted silently and ignored.
        """
        if self._in_list_mode():
            self._command_wait(':LIST OFF')
            self._list_state = 0

        sweep = self._scan_frequencies
        if isinstance(sweep, tuple) and len(sweep) == 3:
            start, stop, steps = float(sweep[0]), float(sweep[1]), int(sweep[2])
        else:
            points = np.asarray(sweep, dtype=np.float64)
            start, stop, steps = float(points[0]), float(points[-1]), len(points)

        dwell = 1.0 / float(self._scan_sample_rate) if self._scan_sample_rate else 1e-3
        if dwell < 100e-6:
            dwell = 100e-6

        self._command_wait(':TRIG:SOUR 2')      # timer
        self._command_wait(':INIT:CONT OFF')    # triggered run mode
        self._command_wait(':TRIG:ADV ONC')     # one trigger = whole sweep

        self._command_wait(':POWer {}'.format(self._scan_power))
        self._command_wait(':FRSW:STAR {}'.format(int(start)))
        self._command_wait(':FRSW:STOP {}'.format(int(stop)))
        self._command_wait(':FRSW:STEP {}'.format(steps))
        self._command_wait(':FRSW:TIME {}'.format(dwell))
        self._command_wait(':FRSW:DIR NORM')
        self._command_wait(':FRSW ON')

    def _set_trigger_edge(self):
        edge = 'POS' if self._rising_edge_trigger else 'NEG'
        self._command_wait(':TRIG:EDG {}\n'.format(edge))
    