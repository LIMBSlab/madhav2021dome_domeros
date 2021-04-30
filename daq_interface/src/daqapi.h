/*!
 * \brief Class to provide high-level function for National Instruments DAQ using
 * the Analogy drivers, usually included with the Xenomai real-time development framework
 *
 * \author $Authors: Robert Nickl, Manu S. Madhav$
 * \date $Last update: 2014-May-24$
 * \contact manusmad@gmail.com
 *
 */

#ifndef DAQAPI_H
#define DAQAPI_H

#include<iostream>
#include<stdio.h>
#include<unistd.h>
#include<sys/mman.h>
#include<errno.h>
#include<vector>
#include<array>
#include<cstdlib>
#include<cstring>
#include<rtdm/analogy.h>
#include"ni_tio_defs.h"

#define DEVICE "analogy0"
#define PULSEWIDTH 150000
#define CR_PACK(chan, rng, aref)					\
	((((aref)&0x3)<<24) | (((rng)&0xff)<<16) | (chan))

class daqapi 
{
    public:
        //! Pathname for device  (e.g. analogy0)
        static char* driverFile;

        //! Device descriptor structure
        a4l_desc_t dsc;		                    

        //! Subdevice indices
        int idx_subd_AI, idx_subd_AO, idx_subd_DIO, idx_subd_CNT[3];

        //! Number of I/O channels
        int num_AI, num_AO, num_DIO;

        //! Size (in bits) of analog I/O channels
        int size_AI, size_AO;

        //! Range index of analog I/O channels
        int rng_AI, rng_AO;

        //! Subdevice info structures
        a4l_sbinfo_t *sbinfo_AO, *sbinfo_AI, *sbinfo_DIO, *sbinfo_CNT[3]; 

        //! Channel info structure
        a4l_chinfo_t *chinfo_AO, *chinfo_AI;

        //! Range info structure
        a4l_rnginfo_t *rnginfo_AO, *rnginfo_AI;

        //! Scan size
        unsigned int scan_size_AO;

        //! Input / Output functions
        double analogRead(int);	
        void configDigitalIn(int);
        int digitalRead(int);
        int digitalIO(unsigned int, unsigned int);
        void analogWrite(int, double); 
        void digitalWrite(int, bool);
        void configDigitalOut(int);

        //! Counter functions
        int disarmCounter(unsigned int);
        int armCounter(unsigned int);
        int resetCounter(unsigned int);
        //void setQuadratureCounterMode(unsigned, int);
        int set_gate_source(unsigned int, unsigned, unsigned);
        int set_other_source(unsigned int, unsigned, unsigned);
        int set_counter_mode (unsigned int, unsigned);
        int setClockSource(unsigned int, int);
        int setupCounter(unsigned int, int, int, int);
        double readCounter(unsigned int);

        //! Constructors and destructor
        daqapi(const char*);
        ~daqapi();

    protected:
        //! Find subdevices and set up subdevices and channels
        void findSubdevices(void);

        //! Translate error codes into human readable form
        int errorHandler(int);
}; 
#endif
