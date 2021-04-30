/*!	
 *  daqapi.cpp
 *	Author:  Robert Nickl, Manu Madhav
 *	Last update: 5/25/14
 */
#include "daqapi.h"

using namespace std;

/*! \brief Constructor for the daqapi class.
*
*   Opens the analogy device, fills the device descriptor and 
*   finds the input / output and counter subdevices.
*/
daqapi::daqapi(const char* analogyDevice)
{	
    //! Open device
    errorHandler(a4l_open(&dsc, analogyDevice));

    //! Allocate buffer for descriptor
    dsc.sbdata = malloc(dsc.sbsize);
    if (dsc.sbdata == NULL)
        errorHandler(-ENOMEM);

    //! Fill descriptor
    errorHandler(a4l_fill_desc(&dsc));

    //! Find the necessary subdevices
    findSubdevices();

    //setup_AO_Ch(); // (RN - 1/15/14) -- It seems we canNOT assume all AO / AI channels are uniform within type
    //setup_AI_Ch();
}

/*! \brief Destructor for the daqapi class
*
*   Frees the information buffer and closes the analogy device
*/
daqapi::~daqapi()
{
    //! Free the information buffer
    if (dsc.sbdata != NULL)
        free(dsc.sbdata);

    //! Close device
    int close_err;
    close_err = a4l_close(&dsc);
}

/*! \brief Find appropriate subdevices
*
*   For the analog input, analog output, counter and digital input / output subdevices,
*   find the index of the subdevices, and retrieve the information structure for each subdevice.
*   Also issues warnings if the subdevices are not found.
*/
void daqapi::findSubdevices(void)
{
    //! Get info for all subdevices 
    idx_subd_AI = idx_subd_AO = idx_subd_DIO = idx_subd_CNT[0] = idx_subd_CNT[1] = idx_subd_CNT[2] = -1;
    a4l_sbinfo_t *sbinfo;
    for (int i=0; i<dsc.nb_subd; i++)
    {
        errorHandler(a4l_get_subdinfo(&dsc, i, &sbinfo));

        //! Digital I/O subdevices
        if ((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_DIO && idx_subd_DIO==-1)
        {
            idx_subd_DIO = i;
            errorHandler(a4l_get_subdinfo(&dsc, i, &sbinfo_DIO));
        }
        //! Analog Input subdevices
        else if ((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_AI && idx_subd_AI==-1)
        {
            idx_subd_AI = i;
            errorHandler(a4l_get_subdinfo(&dsc, i, &sbinfo_AI));
        }
        //! Analog Output subdevices
        else if ((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_AO && idx_subd_AO==-1)
        {
            idx_subd_AO = i;
            errorHandler(a4l_get_subdinfo(&dsc, i, &sbinfo_AO));
        }
        //! Counter subdevices
        else if ((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_COUNTER)
        {
            if (idx_subd_CNT[0]==-1)
            {
                idx_subd_CNT[0] = i;
                errorHandler(a4l_get_subdinfo(&dsc, i, &sbinfo_CNT[0]));
            }
            else if (idx_subd_CNT[1]==-1)
            {
                idx_subd_CNT[1] = i;
                errorHandler(a4l_get_subdinfo(&dsc, i, &sbinfo_CNT[1]));
            }
            else if (idx_subd_CNT[2]==-1)
            {
                idx_subd_CNT[2] = i;
                errorHandler(a4l_get_subdinfo(&dsc, i, &sbinfo_CNT[2]));
            }
        }
    }

    //! Print warnings if any of the subdevices are not found
    if (idx_subd_DIO == -1)
        fprintf(stderr,"Warning: Digital I/O subdevice not found");
    if (idx_subd_AO == -1)
        fprintf(stderr,"Warning: Analog output subdevice not found");
    if (idx_subd_AI == -1)
        fprintf(stderr,"Warning: Analog input subdevice not found");
    if (idx_subd_CNT[0] == -1 && idx_subd_CNT[1] == -1 && idx_subd_CNT[2] == -1)
        fprintf(stderr,"Warning: No counter subdevice found");

    //! Set the data size to read / write
    num_DIO = errorHandler(a4l_sizeof_subd(sbinfo_DIO));
    
    num_AI = (int)(sbinfo_AI -> nb_chan);
    num_AO = (int)(sbinfo_AO -> nb_chan);   

    /* TODO - Different analog channels may have different descriptors / sizes / ranges */
    errorHandler(a4l_get_chinfo(&dsc, idx_subd_AI, 0, &chinfo_AI));
    size_AI = errorHandler(a4l_sizeof_chan(chinfo_AI));
    errorHandler(a4l_get_rnginfo(&dsc, idx_subd_AI,  0, 0, &rnginfo_AI));

    errorHandler(a4l_get_chinfo(&dsc, idx_subd_AO,  0, &chinfo_AO));
    size_AO = errorHandler(a4l_sizeof_chan(chinfo_AO));
    errorHandler(a4l_get_rnginfo(&dsc, idx_subd_AO,  0, 0, &rnginfo_AO));
    scan_size_AO = (chinfo_AO->nb_bits % 8 == 0) ? chinfo_AO->nb_bits / 8 : (chinfo_AO->nb_bits / 8) + 1;
}

/*! \brief Translate return codes into human-readable text for easier debugging
*   
*   The return codes have defined constants in the analogy header files, but they are
*   still not descriptive. It is useful to output full sentence error descriptions instead.
*/
int daqapi::errorHandler(int retcode)
{
    if (retcode>=0)
        return retcode;
    else
    {
        switch (retcode)
        {
            case -EINVAL:
                fprintf(stderr, "Error %d: Missing or wrong argument\n",retcode);
                break;
            case -EFAULT:
                fprintf(stderr, "Error %d: User <--> Kernel transfer error\n",retcode);
                break;
            case -ENOMEM:
                fprintf(stderr, "Error %d: Out of memory\n",retcode);
                break;
            case -ENODEV:
                fprintf(stderr, "Error %d: Incoherent descriptor\n",retcode);
                break;
            case -ENOENT:
                fprintf(stderr, "Error %d: Reading subdevice idle (No command sent)\n",retcode);
                break;
            case -EINTR:
                fprintf(stderr, "Error %d: Calling task unblocked by signals\n",retcode);
                break;
            case -EIO:
                fprintf(stderr, "Error %d: Subdevice cannot handle command\n",retcode);
                break;
            case -EBUSY:
                fprintf(stderr, "Error %d: Subdevice is already processing an asynchronous operation\n",retcode);
                break;
            case -ENOSYS:
                fprintf(stderr, "Error %d: Driver does not provide any handler for instruction bits\n",retcode);
                break;
            default:
                fprintf(stderr, "Error %d: Unknown error\n",retcode);
        }
        exit(0);
    }
}

/*! \brief Analog read function
*   
*   Read the value from a particular channel in the analog input subdevice
*
*   @param idx_chan_AI Analog Input channel
*/
double daqapi::analogRead(int idx_chan_AI)
{		
    unsigned long rawIn;
    double val;

    //rng_AI = errorHandler(a4l_find_range(&dsc, idx_subd_AI, idx_chan_AI, A4L_RNG_VOLT_UNIT, -10, 10, &rnginfo_AI));
    errorHandler(a4l_sync_read(&dsc, idx_subd_AI, PACK(CHAN(idx_chan_AI),0,AREF_DIFF), 0, &rawIn, size_AI));
    //errorHandler(a4l_sync_read(&dsc, idx_subd_AI, CHAN(idx_chan_AI), 0, &rawIn, size_AI));	// 0 ns delay

    //val = (((double)rawIn-65535/2)/65535*22);
    a4l_rawtod(chinfo_AI,rnginfo_AI,&val,&rawIn,1);
    
    return val;
}

/*! \brief Analog write function
*   
*   Write a value to a particular channel in the analog output subdevice
*
*   @param idx_chan_AO  Analog Output channel
*   @param dvalue       Value to write in volts
*/
void daqapi::analogWrite(int idx_chan_AO, double dvalue)
{
    unsigned int rawOut;
    int dtoraw_stat;

    // Convert value to raw data
    dtoraw_stat = a4l_dtoraw(chinfo_AO, rnginfo_AO, &rawOut, &dvalue, 1);

    // Handle little endian case with bit range < 32
    if (scan_size_AO == sizeof(char))
        rawOut *= 0x01010101;
    else if (scan_size_AO == sizeof(short))
        rawOut *= 0x00010001;

    if(dtoraw_stat >= 0) {
        errorHandler(a4l_sync_write(&dsc, idx_subd_AO, CHAN(idx_chan_AO), 0, &rawOut, scan_size_AO));	// 0 ns delay
	}
}

/*! \brief Configure Digital I/O subdevice channel as output
*   
*   Need to configure this before using this channel to write to
*
*   @param chan  Digital channel
*/
void daqapi::configDigitalOut(int chan)
{
    if (idx_subd_DIO > 0)
        errorHandler(a4l_config_subd(&dsc, idx_subd_DIO, A4L_INSN_CONFIG_DIO_OUTPUT, chan));
    else
        fprintf(stderr,"DIO subdevice unknown");
}

/*! \brief Configure Digital I/O subdevice channel as input
*   
*   Need to configure this before using this channel to read from
*
*   @param chan  Digital channel
*/
void daqapi::configDigitalIn(int chan)
{
    if (idx_subd_DIO > 0)
        errorHandler(a4l_config_subd(&dsc, idx_subd_DIO, A4L_INSN_CONFIG_DIO_INPUT, chan));
    else
        fprintf(stderr,"DIO subdevice unknown");
}

/*! \brief Write value to digital I/O channel
*   
*  First configures the digital channel as output, and then writes the boolean bit to the channel 
*
*   @param chan     Digital channel
*   @param on       On / Off
*/
void daqapi::digitalWrite(int chan, bool on)
{
    if (idx_subd_DIO < 0)
    {
        fprintf(stderr,"DIO subdevice unknown");
        exit(0);
    }

    //! Configure as output
    configDigitalOut(chan);

    unsigned int mask = 0;
    unsigned int value = 0;
    mask |= (1<<chan); //< Set chan-th bit for modification

    if (on)
        value |= (1<<chan);
    else
        value &= !(1<<chan);

    digitalIO(mask,value);
}

/*! \brief Reads value from digital I/O channel
*   
*  First configures the digital channel as input, and then reads a boolean bit from the channel 
*
*   @param chan     Digital channel
*/
int daqapi::digitalRead(int chan)
{
    if (idx_subd_DIO < 0)
    {
        fprintf(stderr,"DIO subdevice unknown");
        exit(0);
    }

    configDigitalIn(chan);

    unsigned int mask = 0;
    unsigned int value = 0;

    value = digitalIO(mask,value);
    return (value >> chan) & 1;
}

/*! \brief Low-level function to perform both digital input and output 
*   
*   Uses the mask and value functions to perform synchronous IO call to the digital I/O subdevice
*
*   @param mask     Sets the bit(s) for modification or read
*   @param value    Values for modification or read
*/
int daqapi::digitalIO(unsigned int mask, unsigned int value)
{ 
    //! Handle little endian case with scan size < 32 
    if (num_DIO == sizeof(uint8_t)) {
        mask *= 0x01010101;
        value *= 0x01010101;
    }
    else if (num_DIO == sizeof(uint16_t)) {
        mask *= 0x00010001;
        value *= 0x00010001;
    }

    //! Synchronous DIO call
    errorHandler(a4l_sync_dio(&dsc, idx_subd_DIO, &mask, &value));

    if (num_DIO == sizeof(uint8_t)) {
        value = *((uint8_t *)&value);
    }
    else if (num_DIO == sizeof(uint16_t)) {
        value = *((uint16_t *)&value);
    }

    return value;
}

int daqapi::armCounter(unsigned int chan)
{
    unsigned int data[2];
    a4l_insn_t insn;

    // arm the counter
    insn.type      = A4L_INSN_CONFIG;
    insn.idx_subd  = idx_subd_CNT[chan];
    insn.chan_desc = 0;
    insn.data_size = sizeof(data[0])*2;

    data[0]      = A4L_INSN_CONFIG_ARM;
    data[1]      = NI_GPCT_ARM_IMMEDIATE;
    insn.data      = data;

    return errorHandler(a4l_snd_insn(&dsc, &insn));
}

int daqapi::disarmCounter(unsigned int chan)
{
    unsigned int data[2];
    a4l_insn_t insn;

    // arm the counter
    insn.type      = A4L_INSN_CONFIG;
    insn.idx_subd  = idx_subd_CNT[chan];
    insn.chan_desc = 0;
    insn.data_size = sizeof(data[0])*2;

    data[0]      = A4L_INSN_CONFIG_DISARM;
    data[1]      = NI_GPCT_ARM_IMMEDIATE;
    insn.data      = data;

    return errorHandler(a4l_snd_insn(&dsc, &insn));
}

int daqapi::set_gate_source (unsigned int chan, unsigned int gate_index, unsigned int gate_source)
{
    unsigned int data[3];
    a4l_insn_t insn;

    //gate_source = NI_GPCT_DISABLED_GATE_SELECT;//NI_GPCT_PFI_GATE_SELECT(12)//CR_EDGE for rising edge
    insn.type      = A4L_INSN_CONFIG;
    insn.idx_subd  = idx_subd_CNT[chan];
    insn.chan_desc = 0;
    insn.data_size = sizeof(data[0])*3;

    data[0]      = A4L_INSN_CONFIG_SET_GATE_SRC;
    data[1] = gate_index;
    data[2] = gate_source;
    insn.data      = data;

    return errorHandler(a4l_snd_insn(&dsc, &insn));
}

int daqapi::set_other_source(unsigned int chan, unsigned int index, unsigned int source)
{
    unsigned int data[3];
    a4l_insn_t insn;

    insn.type = A4L_INSN_CONFIG;
    insn.idx_subd = idx_subd_CNT[chan];
    insn.chan_desc = 0;
    insn.data_size = sizeof(data[0])*3;

    data[0] = A4L_INSN_CONFIG_SET_OTHER_SRC;
    data[1] = index;
    data[2] = source;
    insn.data = data;

    return errorHandler(a4l_snd_insn(&dsc, &insn));
}

int daqapi::set_counter_mode (unsigned int chan, unsigned int mode_bits)
{
    unsigned int data[2];
    a4l_insn_t insn;

    insn.type=A4L_INSN_CONFIG;
    insn.idx_subd = idx_subd_CNT[chan];
    insn.chan_desc = 0;
    insn.data_size= 2*sizeof(data[0]);

    data[0] = A4L_INSN_CONFIG_SET_COUNTER_MODE;
    data[1] = mode_bits;
    insn.data = data;

    return errorHandler(a4l_snd_insn(&dsc, &insn));
}

int daqapi::resetCounter(unsigned int chan)
{
    unsigned int data[1];
    a4l_insn_t insn;

    // reset counter
    insn.type      = A4L_INSN_CONFIG;
    insn.idx_subd  = idx_subd_CNT[chan];
    insn.chan_desc = 0;
    insn.data_size = sizeof(data[0]);
    data[0]      = A4L_INSN_CONFIG_RESET;
    insn.data      = data;

    return errorHandler(a4l_snd_insn(&dsc, &insn));
}

int daqapi::setClockSource(unsigned int chan, int source_channel_pfi)
{
    unsigned int data[3];
    a4l_insn_t insn;

    //set source pulse input
    insn.type      = A4L_INSN_CONFIG;
    insn.idx_subd  = idx_subd_CNT[chan];
    insn.chan_desc = 0;
    insn.data_size = sizeof(data[0])*3;
    data[0]      = A4L_INSN_CONFIG_SET_CLOCK_SRC;
    //data[1]      = NI_GPCT_PFI_CLOCK_SRC_BITS(source_channel_pfi);//NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS//e.g. defined in ni_tio.h // Gi_Source_Bit in 4-63 in DAQ-STC manual 8(?) works for 0,1,2
    data[1]      = NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS;//e.g. defined in ni_tio.h // Gi_Source_Bit in 4-63 in DAQ-STC manual 8(?) works for 0,1,2
    data[2]      = 0;
    insn.data    = data;

    return errorHandler(a4l_snd_insn(&dsc, &insn));
}

int daqapi::setupCounter(unsigned int chan, int source_a, int source_b, int source_z)
{
    unsigned int counter_mode;	// counter setup bits
    unsigned int initial_count = 0;

    disarmCounter(chan);
    resetCounter(chan);

    //set initial counter to 0
    errorHandler(a4l_sync_write(&dsc, idx_subd_CNT[chan], 0, 0, &initial_count, sizeof(initial_count)));
    errorHandler(a4l_sync_write(&dsc, idx_subd_CNT[chan], 1, 0, &initial_count, sizeof(initial_count)));
    errorHandler(a4l_sync_write(&dsc, idx_subd_CNT[chan], 2, 0, &initial_count, sizeof(initial_count)));

    setClockSource(chan,source_a);

    source_a = NI_GPCT_PFI_OTHER_SELECT(source_a);
    source_b = NI_GPCT_PFI_OTHER_SELECT(source_b);

    if (source_z >= 0) {
        source_z = NI_GPCT_PFI_OTHER_SELECT(source_z);

        set_gate_source(chan,0, source_z);
        set_gate_source(chan,1, source_z);

        set_other_source(chan,NI_GPCT_SOURCE_ENCODER_Z, source_z);
    }
    else {
        set_gate_source(chan,0,NI_GPCT_DISABLED_GATE_SELECT);
        set_gate_source(chan,1,NI_GPCT_DISABLED_GATE_SELECT);
    }

    set_other_source(chan,NI_GPCT_SOURCE_ENCODER_A, source_a);
    set_other_source(chan,NI_GPCT_SOURCE_ENCODER_B, source_b);

    // counter mode bits
    //counter_mode = NI_GPCT_COUNTING_MODE_NORMAL_BITS;
    counter_mode = NI_GPCT_COUNTING_MODE_QUADRATURE_X4_BITS;
    // count up and down determined by Up/Down signal
    counter_mode |= NI_GPCT_COUNTING_DIRECTION_HW_UP_DOWN_BITS; //P0.6 to determine up down
    //Don't alternate the reload source between the load a and load b registers.
    //counter_mode |= NI_GPCT_RELOAD_SOURCE_FIXED_BITS;
    // start and stop on gate bit == only count when gate is 1
    //counter_mode |= NI_GPCT_EDGE_GATE_STARTS_STOPS_BITS;
    // don't disarm on terminal count or gate signal
    //counter_mode |= NI_GPCT_NO_HARDWARE_DISARM_BITS;
    //if (source_z >= 0) {
        //reload at gate input
        //counter_mode |= NI_GPCT_LOADING_ON_GATE_BIT;
        // index bits
        //counter_mode |= NI_GPCT_INDEX_ENABLE_BIT;
        //counter_mode |= NI_GPCT_INDEX_PHASE_LOW_A_HIGH_B_BITS;
    //}
    
    set_counter_mode(chan,counter_mode);

    armCounter(chan);

    return 0;
}

double daqapi::readCounter(unsigned int chan)
{
    signed counter;
    errorHandler(a4l_sync_read(&dsc, idx_subd_CNT[chan], 0, 0, &counter, 4));

    return (double)(counter);
}
