#include "Whole_EtherCAT_DC_2.h"

// #ifndef ELMOETHERCAT_SOURCE
// #define ELMOETHERCAT_SOURCE
// using namespace std;


////// Define static variables //////
char EthercatMaster::IOmap[4096];

#if NUM_ELMO != 1
	ELMO_GOLD EthercatMaster::_elmo_gold[NUM_ELMO];
#else
	ELMO_GOLD EthercatMaster::_elmo_gold;
#endif
#if NUM_MY != 1
	MY EthercatMaster::_my[NUM_MY];
#else
	MY EthercatMaster::_my;
#endif

// ////// Define static variables //////
int EthercatMaster::ControlWord = 0;
int EthercatMaster::controlword = 0;
int EthercatMaster::expectedWKC = 0;

int EthercatMaster::dorun = 0;
int EthercatMaster::okop = 0;
int64_t EthercatMaster::toff = 0LL;
uint64_t EthercatMaster::overruns = 0LL;
// int ms(double _time);

int ms(double _time)
{
    return (int)(_time*1000000);
}

// unsigned int EthercatMaster::cycle_ns_ethercat = 1000000; /* 1 ms */
RTIME EthercatMaster::cycle_ns_ethercat = 1000000; /* 1 ms */
////// Define static variables //////


EthercatMaster::EthercatMaster()
{
	std::cout << "\033[1;33m" << "Create EthercatMaster class!!!" << "\033[0m" << std::endl;
}
EthercatMaster::~EthercatMaster()
{
	// // #ifdef _USE_DC
	for (int i=0; i<NUM_ACT; ++i)
		ec_dcsync0(i+1, FALSE, 0, 0); // SYNC0,1 on slave 1

	dorun = 0;
	okop = 0;

	ec_close();

	rt_printf("End simple test, close socket\n");	
	
	std::cout << "\033[1;34m" << "Delete EthercatMaster class!!!" << "\033[0m" << std::endl;

	rt_task_sleep(1e8);
}

/// @brief Set initial EtherCAT communication
/// @param mode_op Control mode of Motor driver
/// @param ifname The name of EtherCAT network
/// @return 1
///		EtherCAT operates well
/// @return 0
///		Error of EtherCAT operation
int EthercatMaster::init(const int8_t mode_op, const char *ifname)
{
    ////// Ethercat SOEM //////
	int j, oloop, iloop, wkc_count, chk;

	if (ec_init(ifname))
    {
        rt_printf("ec_init on %s succeeded.\n",ifname);
		
        /// Network discovery ///
        if ( ec_config_init(FALSE) > 0 )
        {
            rt_printf("%d slaves found and configured.\n", ec_slavecount);

			/// Driver Config ///
			initSlave();

			#if DCMODE == 1
            	ec_configdc();
			#endif

			/// If CA disable, automapping works ///
            ec_config_map(&IOmap);
			// print_my_slave_debug_info(ec_slavecount);

			#if DCMODE == 1
				ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
				ec_readstate();
			#endif

			/// Show slave info ///
			showSlaveInfo();

			/// Wait for all slaves to reach SAFE_OP state ///
			#if DCMODE == 0
				rt_printf("Slaves mapped, state to SAFE_OP.\n");
				ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
			#endif
			
			expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            rt_printf("Calculated workcounter %d\n", expectedWKC);
			
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			
			#if DCMODE == 0
				/// Send one valid process data to make outputs in slaves happy ///
				ec_send_processdata();
				ec_receive_processdata(EC_TIMEOUTRET);
			#endif
			
			/// Request OP state for all slaves ///
            ec_writestate(0);
			chk = 9000;    
			do                                                                                                                                                                                           
            {                                                                                                                                                                                            
              	ec_send_processdata();                                                                                                                                                                    
              	ec_receive_processdata(500);                                                                                                                                                              
             	ec_statecheck(0, EC_STATE_OPERATIONAL, 500);                                                                                                                                              
            }                                                                                                                                                                                            
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

			#if DCMODE == 0
				chk = 40;
			#else
				dorun = 1;
			#endif

			#if DCMODE == 1
				ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE); //5 * EC_TIMEOUTSTATE
			#else
				do
				{
					ec_send_processdata();
					ec_receive_processdata(EC_TIMEOUTRET);
					ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
				}
				while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
			#endif

			#if DCMODE == 0
				registerParam();
				okop = 1;
			#endif

            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
				rt_printf("Operational state reached for all slaves = %d.\n", ec_slave[0].state);
				
				#if DCMODE == 1
					registerParam();
					okop = 1;
				#endif

				testDriveState(mode_op);
				testDriveStateMY(mode_op);

				return 1;
            }
            else
            {
                rt_printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(int i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        rt_printf("[FAIL]Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            rt_printf("No slaves found!\n");
        }
    }
    else
    {
        rt_printf("No socket connection on %s\nExcecute as root\n",ifname);
    }

	return 0;
}


/// @brief Do PDO mapping of motor driver
/// @param slave The slave of motor driver
/// @return retval
///		PDO mapping                                                                                                                                                                           
int Elmosetup(uint16 slave) // must be set up(by HJ)
{
    int retval;
    uint16 u16val;
    
    retval = 0;
	
	uint16 map_1c12[] = {0x0002, 0x1605, 0x161D};
	uint16 map_1c13[] = {0x000A, 0x1A0E, 0x1A11, 0x1A18, 0x1A1C, 0x1A1E,  0x1A1D, 0x1A1F, 0x1A13, 0x1A0A, 0x1A0B}; //32 bite to 8 bite
                    //변수개수, actualPos, actualVel, DC, Digitalout, Auxikuary, Analog, Current, actualTor, status, Mode
	retval +=ec_SDOwrite(slave,0x1c12,0x00,TRUE,sizeof(map_1c12),&map_1c12,EC_TIMEOUTSAFE);
	retval +=ec_SDOwrite(slave,0x1c13,0x00,TRUE,sizeof(map_1c13),&map_1c13,EC_TIMEOUTSAFE);

	#if DCMODE == 1
		ec_dcsync0(slave,   1,    1000000U,   250000U);
		rt_printf("ec_dcsync0 called on slave %u\n",slave);
	#endif

    rt_printf("ELMO slave %d set, retval = %d\n", slave, retval);
    return 1;
}

int MYSetup(uint16 slave) 
{
    int retval;                              
    uint16 u16val;
    
    retval = 0;
	
	uint16 map_1c12[] = {0x0001, 0x1600}; //1600
	uint16 map_1c13[] = {0x0001, 0x1A00}; //1A00

	retval +=ec_SDOwrite(slave,0x1c12,0x00,TRUE,sizeof(map_1c12),&map_1c12,EC_TIMEOUTSAFE);
	retval +=ec_SDOwrite(slave,0x1c13,0x00,TRUE,sizeof(map_1c13),&map_1c13,EC_TIMEOUTSAFE);
	

	#if DCMODE == 1
		// ec_dcsync01(slave,   1,   1000000U,  1000000U,  1000U);
		ec_dcsync0(slave,   1,    1000000U,   250000U);
		rt_printf("ec_dcsync0 called on slave %u\n",slave);
	#endif

	rt_printf("MY slave %d set, retval = %d\n", slave, retval);

    return 1;

}


/// @brief Set intial setting of motor driver
/// @return 
int EthercatMaster::initSlave() 
{
	for(int i = 1 ; i <= NUM_ELMO ; i++)
	{
		if((ec_slave[i].eep_man == ELMO_VENDOR_ID) && (ec_slave[i].eep_id == ELMO_GOLD_PRODUCT_CODE))
		{
			printf("Elmo Found %s at position %d\n", ec_slave[i].name, i);
			/* link slave specific setup to preop->safeop hook */
			ec_slave[i].PO2SOconfig = &Elmosetup;
		}
	}

	for (int i = NUM_ELMO + 1; i<= NUM_ELMO + NUM_MY; i++)   
	{	
		if((ec_slave[i].eep_man == MY_VENDOR_ID) && (ec_slave[i].eep_id == MY_PRODUCT_CODE))
		{
			printf("MY Found %s at position %d\n", ec_slave[i].name, i);
			/* link slave specific setup to preop->safeop hook */
			ec_slave[i].PO2SOconfig = &MYSetup;

			// 구조체 크기 확인
			printf("[DEBUG]MY_IN size  = %lu bytes\n", sizeof(MY_IN));
			printf("[DEBUG]MY_OUT size = %lu bytes\n", sizeof(MY_OUT));
		}
	}


	return 0;
}


/// @brief Show slave state information
/// @return 
int EthercatMaster::showSlaveInfo()
{
	for (int i=1; i<=ec_slavecount; i++) 
	{
		printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
		i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
		ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
	}  
	return 0;
}

/// @brief Set target data in register of ELMO driver
/// @return 
int EthercatMaster::registerParam()
{

	for( int id = 0; id < NUM_ELMO; id++ )
	{
		// ELMO
		if( (ec_slave[id+1].eep_man == ELMO_VENDOR_ID) && (ec_slave[id+1].eep_id == ELMO_GOLD_PRODUCT_CODE) )
		{
			#if NUM_ELMO != 1
				_elmo_gold[id].InParam = (struct ELMO_GOLD_IN *)(ec_slave[id + 1].outputs);
				_elmo_gold[id].OutParam = (struct ELMO_GOLD_OUT *)(ec_slave[id + 1].inputs);
			#else
				_elmo_gold.InParam = (struct ELMO_GOLD_IN *)(ec_slave[id + 1].outputs);
				_elmo_gold.OutParam = (struct ELMO_GOLD_OUT *)(ec_slave[id + 1].inputs);
			#endif

			// rt_printf("ELMO[%d] Output Addr: %p, Input Addr: %p\n", id, ec_slave[id + 1].outputs, ec_slave[id + 1].inputs);
			rt_printf("ELMO driver register setting\n");
		}
	}

	for( int id = 0; id < NUM_MY; id++ ) {
		// MY
		if( (ec_slave[id+1+NUM_ELMO].eep_man == MY_VENDOR_ID) && (ec_slave[id+1+NUM_ELMO].eep_id == MY_PRODUCT_CODE) )
		{
			#if NUM_MY != 1
				_my[id].InParam = (struct MY_IN *)(ec_slave[id + 1 + NUM_ELMO].outputs);
				_my[id].OutParam = (struct MY_OUT *)(ec_slave[id + 1 + NUM_ELMO].inputs);
			#else
				_my.InParam = (struct MY_IN *)(ec_slave[id + 1 + NUM_ELMO].outputs);
				_my.OutParam = (struct MY_OUT *)(ec_slave[id + 1 + NUM_ELMO].inputs);
			#endif
			rt_printf("MY driver register setting\n");

			// #if NUM_MY != 1
			// 	rt_printf("MY[%d] mapped controlword: 0x%X\n", id, _my[id].InParam->controlword);
			// 	rt_printf("MY[%d] mapped targettorque: %d\n", id, _my[id].InParam->targettorque);
			// 	rt_printf("MY[%d] mapped statusword: 0x%X\n", id, _my[id].OutParam->statusword);
			// 	rt_printf("MY[%d] mapped actualtorque: %d\n", id, _my[id].OutParam->torqueactualvalue);
			// #else
			// 	rt_printf("MY[%d] mapped controlword: 0x%X\n", id, _my.InParam->controlword);
			// 	rt_printf("MY[%d] mapped targettorque: %d\n", id, _my.InParam->targettorque);
			// 	rt_printf("MY[%d] mapped statusword: 0x%X\n", id, _my.OutParam->statusword);
			// 	rt_printf("MY[%d] mapped actualtorque: %d\n", id, _my.OutParam->torqueactualvalue);
			// #endif
		}	
	}

    return 0;
}


/// @brief Test motor driver settings
/// @param mode_op The control mode of motor driver
/// @return 
int EthercatMaster::testDriveState(const int8_t mode_op)  
{
	for (int i=0; i<NUM_ELMO; i++) 
	{
		#if NUM_ELMO != 1
			_elmo_gold[i].InParam->ModeOfOperation = mode_op;
			_elmo_gold[i].InParam->MaxTorque = 1000;
			ec_send_processdata();

			ec_receive_processdata(EC_TIMEOUTRET); 
			_elmo_gold[i].InParam->ControlWord = 0x6;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);  
			_elmo_gold[i].InParam->ControlWord = 0x7;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET); 

			_elmo_gold[i].InParam->ControlWord = 0x0F;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET); 
		#else
			_elmo_gold.InParam->ModeOfOperation = mode_op;
			_elmo_gold.InParam->MaxTorque = 1000;
			ec_send_processdata();
			
			ec_receive_processdata(EC_TIMEOUTRET); 
			_elmo_gold.InParam->ControlWord = 0x6;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);  
			_elmo_gold.InParam->ControlWord = 0x7;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET); 
		
			_elmo_gold.InParam->ControlWord = 0x0F;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET); 
		#endif

	}
	return 0;
}

int EthercatMaster::testDriveStateMY(const int8_t mode_op)  
{
    for (int i=0; i<NUM_MY; i++) 
	{
		#if NUM_MY != 1
			_my[i].InParam->modeofoperation = mode_op;
			//_my[i].InParam->maxtorque = 1000;
			ec_send_processdata();
			
			ec_receive_processdata(EC_TIMEOUTRET); 
			_my[i].InParam->controlword = 0x6;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);  
			_my[i].InParam->controlword = 0x7;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET); 

			_my[i].InParam->controlword = 0x0F;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET); 
		#else
			_my.InParam->modeofoperation = mode_op;
			//_my[i].InParam->maxtorque = 1000;
			ec_send_processdata();
			
			ec_receive_processdata(EC_TIMEOUTRET); 
			_my.InParam->controlword = 0x6;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);  
			_my.InParam->controlword = 0x7;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET); 

			_my.InParam->controlword = 0x0F;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET); 
		#endif
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////
///////////for DC MODE///////////////////////////////////////////////////////////////////////
// void EthercatMaster::add_timespec(RTIME *ts, int64 addtime)
/// @brief Add cycle time to current time
/// @param ts Current real time
/// @param addtime One cycle time
void EthercatMaster::add_timespec(RTIME *ts, RTIME addtime)
{
	*ts += addtime;
}

/// @brief Synchronize time between PC and motor driver
/// @param reftime Reference time of motor driver
/// @param cycletime One cycle time
/// @return offsettime 
///		Calculated offset time about EtherCAT communication
void EthercatMaster::ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
	//// PI calculation to get linux time synced to DC time
	static int64 integral = 0;
	int64 delta;
	//// set linux sync point 50us later than DC sync, just as example
	delta = (reftime - 50000) % cycletime;
	if(delta> (cycletime / 2)) { delta= delta - cycletime; }
	if(delta>0){ integral++; }
	if(delta<0){ integral--; }
	// *offsettime = -(delta / 100) - (integral / 20);
	*offsettime = -(delta / 5) - (integral / 10);  //original
	// *offsettime = -(delta / 100) - (integral / 20);
   	// gl_delta = delta;

	// // Debug
	// static int debug_count = 0;

	// if (++debug_count % 500 == 0) {
	// 	rt_printf("[SYNC] delta: %lld, toff: %lld, integral: %lld\n", delta, *offsettime, integral);
	// }
}

// void EthercatMaster::wait_period(RTIME wakeup_count, int count)
/// @brief Wait time until certain time
/// @param wakeup_count Real time of waiting
void EthercatMaster::wait_period(RTIME wakeup_count)
{
    while (1)
    {
        // RTIME wakeup_count = system2count(wakeup_time);
        RTIME current_count = rt_timer_read();
		// RTIME previous, now;

        if (((wakeup_count < current_count)
                || (wakeup_count > current_count + (50 * cycle_ns_ethercat))) && (okop == 1)) {
            rt_printf("\n\n\n\n %s(): unexpected wake time!\n Wakeup: %lld, Current: %lld\n\n\n\n\n", __func__, wakeup_count, current_count);
        }
		// previous = rt_timer_read();
        switch (rt_task_sleep_until(wakeup_count)) {
            case EINTR :
                rt_printf("rt_sleep_until(): RTE_UNBLKD\n");
                continue;

            case EPERM :
                rt_printf("rt_sleep_until(): RTE_TMROVRN\n");
                overruns++;

                if (overruns % 100 == 0) {
                    // in case wake time is broken ensure other processes get
                    // some time slice (and error messages can get displayed)
                    rt_task_sleep(cycle_ns_ethercat / 100);
					
                }
                break;

            default:
				// now = rt_timer_read();
                break;
        }

		// if( count % 1000 == 0 )
		// {
		// 	rt_printf("wakeup_count(ns): %f\n", (double)wakeup_count);
		// 	rt_printf("current_time(ns): %f\n", (double)previous);
		// 	rt_printf("wait_period(ms): %f\n", (double)(now - previous)/1e6);	// ms
		// }
			

        // done if we got to here
        break;
    }

    // calc next wake time (in sys time)
    // wakeup_time += cycle_ns_ethercat;
}

/// @brief Set loop time of Sensory feedback task
/// @param _freq Loop frequency (Unit : Hz)
void EthercatMaster::SetLoopTime(double _freq)
{
    double _ms = 1000.0/_freq;
    dtCtrl = 1.0/_freq;

    #if DCMODE == 0
        rt_task_set_periodic(NULL, TM_NOW, (int)ms(_ms));
    #elif DCMODE == 1
        InitDCMODE(_ms);
    #endif
}

/// @brief Set loop time of EtherCAT check task
/// @param _freq Loop frequency (Unit : Hz)
void EthercatMaster::SetLoopTimeCheck(double _freq)
{
    double _ms = 1000.0/_freq;

    #if DCMODE == 0
        rt_task_set_periodic(NULL, TM_NOW, (int)ms(_ms));
    #elif DCMODE == 1
        clock_gettime(CLOCK_MONOTONIC, &ecatcheckTime);
        cycletimeCheck = _ms*(unsigned int)msConvert;
    #endif
}

/// @brief EtherCAT task
void EthercatMaster::RunEtherCATCheck()
{
    #if DCMODE == 1
        ecatcheckTime.tv_nsec += cycletimeCheck; // frequency : 1kHz (1 ms)
        if( ecatcheckTime.tv_nsec > SEC_IN_NSEC )
        {
            ecatcheckTime.tv_nsec -= SEC_IN_NSEC;
            ecatcheckTime.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ecatcheckTime, NULL);
    #endif

    previousCheck = rt_timer_read();

    ////// Check EtherCAT state and turn on EtherCAT state forcibly if it turns off
    // CheckEtherCATState();
    
    nowCheck = rt_timer_read();
    timet_ecat = (double)((nowCheck - previousCheck)/1e9);

    #if DCMODE == 0
        rt_task_wait_period(NULL);
    #endif
}

/// @brief Wait loop time
void EthercatMaster::WaitTime()
{
    #if DCMODE == 1
        tsbefore = ts;

        ////// Update ts with cycle time and delay time of EtherCAT 
        ////// ts : task start time of previous loop
        ////// cycletime : one loop time of EtherCAT task
        ////// ethercatMaster.toff : delay time of EtherCAT communication
        add_timespec(&ts, cycletime + (RTIME)toff);

        clock_gettime(CLOCK_MONOTONIC, &waitperiod_check_time);

        if(waitperiod_flag == 0)
        {
            rt_printf("ts time   : %f\n", (double)ts);
            rt_printf("rt time   : %f\n\n", (double)rt_timer_read());
            rt_printf("ts - rt: %f       \n\n", ((double)ts - (double)rt_timer_read()) / 1e6);

            if( ts > rt_timer_read() &&  okop == 1 )
                waitperiod_flag = 1;
        }

        if( ts < rt_timer_read() &&  okop == 1)
        {
            waitperiod_flag = 0;
            rt_printf("wait period exception!");
        }

        ////// Wait EtherCAT task until reaching ts
        ////// ts : Predicted finish time of previous loop
        wait_period(ts);
    #else
        rt_task_wait_period(NULL);
    #endif
}

#if DCMODE == 1
    /// @brief Initialize loop time of sensory feedback task
    /// @param _ms Loop time (Unit : ms)
    void EthercatMaster::InitDCMODE(double _ms)
    {
        cycletime = (RTIME)(_ms*msConvert);
        ts = rt_timer_read();
        ht = (ts / msConvert) + 1; /* round to nearest ms */
        ts = ht * msConvert;
        ec_send_processdata();
    }
#endif
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

/// @brief Send target raw data to motor driver
void EthercatMaster::processRxDomain()
{
	// TODO: Write control data from axes to servos via master
	if(okop == 1){
		for (int i=0; i<NUM_ELMO; ++i)
		{
			// elmoservoOn(i);
			servoOn(i, "ELMO");
			
		}
		for (int i=0; i<NUM_MY; ++i)
		{
			// myservoOn(i);
			servoOn(i, "MY");
			
		}
	}
	ec_send_processdata(); 
}

/// @brief Receive raw data from motor driver
/// @return Value
///		Received data from motor driver
int EthercatMaster::processTxDomain()
{
	// wait time for 500 us until slave sends EtherCAT communication 
	// return ec_receive_processdata(EC_TIMEOUTRET);		
	return ec_receive_processdata(50);
}

/// @brief Write raw data in each buffer
/// @param EntryID ID of raw data
/// @param data Raw data
void EthercatMaster::writeBuffer(const int EntryID, void* data)
{
    switch (EntryID)
	{
		// case TARGET_POSITION:
		// {
		// 	int32_t* _targetPosition = static_cast<int32_t *>(data);
        //     for (int i=0; i<NUM_ELMO; i++)
        //     {
        //         _elmo_gold[i].InParam->TargetPosition = _targetPosition[i];
        //     }
		// }
		// 	break;						

		// case TARGET_VELOCITY:
		// {
		// 	int32_t * const _targetVelocity = static_cast<int32_t * const>(data);
        //     for (int i=0; i<NUM_ELMO; i++)
        //     {
        //         _elmo_gold[i].InParam->TargetVelocity = _targetVelocity[i];
        //     }
		// }
		// 	break;						

		case TARGET_TORQUE:
		{
			int16_t * const _targetTorque = static_cast<int16_t * const>(data);		
            for (int i=0; i<NUM_ELMO; i++)
            {
				#if NUM_ELMO != 1
                	_elmo_gold[i].InParam->TargetTorque = _targetTorque[i];
				#else
					_elmo_gold.InParam->TargetTorque = _targetTorque[i];
				#endif
            }
		}
			break;						

		case MAX_TORQUE:
		{
			uint16_t * const _maxTorque = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
				#if NUM_ELMO != 1
                	_elmo_gold[i].InParam->MaxTorque = _maxTorque[i];
				#else
					_elmo_gold.InParam->MaxTorque = _maxTorque[i];
				#endif
            }
		}
			break;						

		case CONTROL_WORD:
		{
			uint16_t * const _controlWord = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
				#if NUM_ELMO != 1
                	_elmo_gold[i].InParam->ControlWord = _controlWord[i];
				#else
					_elmo_gold.InParam->ControlWord = _controlWord[i];
				#endif
            }
		}
			break;						

		case MODE_OF_OPERATION:
		{
			int8_t * const _modeOfOperation = static_cast<int8_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
				#if NUM_ELMO != 1
                	_elmo_gold[i].InParam->ModeOfOperation = _modeOfOperation[i];
				#else
					_elmo_gold.InParam->ModeOfOperation = _modeOfOperation[i];
				#endif
            }
		}
			break;

		case DIGITAL_OUTPUTS:
		{
			int32_t * const _digitalOuputs = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
				#if NUM_ELMO != 1
                	_elmo_gold[i].InParam->DigitalOutputs = _digitalOuputs[i];
				#else
					_elmo_gold.InParam->DigitalOutputs = _digitalOuputs[i];
				#endif
            }
		}
			break;


		//MY EtherCAT
		case CONTROLWORD_my:
		{
			uint16_t * const _ControlWord = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_my[i].InParam->controlword = _ControlWord[i];
				#else
					_my.InParam->controlword = _ControlWord[i];
				#endif
            }
		}
			break;

		case TARGETPOSITION_my:
		{
			int32_t * const _TargetPosition= static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_my[i].InParam->targetposition = _TargetPosition[i];
				#else
					_my.InParam->targetposition = _TargetPosition[i];
				#endif
            }
		}
			break;

		case TARGETVELOCITY_my:
		{
			int32_t * const _TargetVelocity= static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_my[i].InParam->targetvelocity = _TargetVelocity[i];
				#else
					_my.InParam->targetvelocity = _TargetVelocity[i];
				#endif
            }
		}
			break;

		case TARGETTORQUE_my:
		{
			int16_t * const _TargetTorque= static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_my[i].InParam->targettorque = _TargetTorque[i];
				#else
					_my.InParam->targettorque = _TargetTorque[i];
				#endif
            }
		}
			break;

		case MAXTORQUE_my:
		{
			uint16_t * const _Maxtorque= static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_my[i].InParam->maxtorque = _Maxtorque[i];
				#else
					_my.InParam->maxtorque = _Maxtorque[i];
				#endif
            }
		}
			break;

		case MODEofOPERATION_my:
		{
			int8_t * const _ModeofOperation= static_cast<int8_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_my[i].InParam->modeofoperation = _ModeofOperation[i];
				#else
					_my.InParam->modeofoperation = _ModeofOperation[i];
				#endif
            }
		}
			break;
								
							
		default:	// Undefined Entry ID	
			
			break;
	}
}

/// @brief Read raw data from buffer
/// @param EntryID ID of raw data
/// @param data Raw data
void EthercatMaster::readBuffer(const int EntryID, void* data)
{
    switch (EntryID)
	{		
		case POSITION_ACTUAL_VALUE:
		{
            int32_t * _positionActualValue = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
				#if NUM_ELMO != 1
                	_positionActualValue[i] = _elmo_gold[i].OutParam->PositionActualValue;
				#else
					_positionActualValue[i] = _elmo_gold.OutParam->PositionActualValue;
				#endif            
			}
			
		}
			break;			

        case VELOCITY_ACTUAL_VALUE:
		{
			int32_t * _velocityActualValue = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_velocityActualValue[i] = _elmo_gold[i].OutParam->VelocityActualValue;
				#else
					_velocityActualValue[i] = _elmo_gold.OutParam->VelocityActualValue;
				#endif
            }
		}
			break;	

        case DC_LINK_CIRCUIT_VOLTAGE:
		{
			uint32_t * _dCLinkCircuitVoltage = static_cast<uint32_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_dCLinkCircuitVoltage[i] = _elmo_gold[i].OutParam->DCLinkCircuitVoltage;
				#else
					_dCLinkCircuitVoltage[i] = _elmo_gold.OutParam->DCLinkCircuitVoltage;
				#endif
            }
		}
			break;	

        case DIGITAL_INPUTS:
		{
			uint32_t * _digitalInputs = static_cast<uint32_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_digitalInputs[i] = _elmo_gold[i].OutParam->DigitalInputs;
				#else
					_digitalInputs[i] = _elmo_gold.OutParam->DigitalInputs;
				#endif
            }
		}
			break;	

        case AUXILIARY_POSITION_ACTUAL_VALUE:
		{
			int32_t * _auxiliaryPositionActualValue = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_auxiliaryPositionActualValue[i] = _elmo_gold[i].OutParam->AuxiliaryPositionActualValue;
				#else
					_auxiliaryPositionActualValue[i] = _elmo_gold.OutParam->AuxiliaryPositionActualValue;
				#endif
            }
		}
			break;	

        case ANALOG_INPUT_1:
		{
			int16_t * _analogInput1 = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_analogInput1[i] = _elmo_gold[i].OutParam->AnalogInput1;
				#else
					_analogInput1[i] = _elmo_gold.OutParam->AnalogInput1;
				#endif
	
            }
		}
			break;				

		case CURRENT_ACTUAL_VALUE:
		{
			int16_t * _currentActualValue = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_currentActualValue[i] = _elmo_gold[i].OutParam->CurrentActualValue;
				#else
					_currentActualValue[i] = _elmo_gold.OutParam->CurrentActualValue;
				#endif
            }
		}
		break;	

		case TORQUE_ACTUAL_VALUE:
		{
			int16_t * _torqueActualValue = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_torqueActualValue[i] = _elmo_gold[i].OutParam->TorqueActualValue;
				#else
					_torqueActualValue[i] = _elmo_gold.OutParam->TorqueActualValue;
				#endif
            }
		}
			break;			

		case STATUS_WORD:
		{
			uint16_t * _statusWord = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_statusWord[i] = _elmo_gold[i].OutParam->StatusWord;
				#else
					_statusWord[i] = _elmo_gold.OutParam->StatusWord;
				#endif
            }
		}
			break;			

		case MODE_OF_OPERATION_DISPLAY:
		{
			int8_t * _modeOfOperationDisplay = static_cast<int8_t * const>(data);
            for (int i=0; i<NUM_ELMO; i++)
            {
                #if NUM_ELMO != 1
                	_modeOfOperationDisplay[i] = _elmo_gold[i].OutParam->ModeOfOperationDisplay;
				#else
					_modeOfOperationDisplay[i] = _elmo_gold.OutParam->ModeOfOperationDisplay;
				#endif
            }
		}
			break;						
		
		//MY EhterCAT
		case STATUSWORD_my:
		{
			uint16_t * _Statusword = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_Statusword[i] = _my[i].OutParam->statusword;
				#else
					_Statusword[i] = _my.OutParam->statusword;
				#endif
            }
		}
		break;		

		case POSITION_ACTUALVALUE_my:
		{
			int32_t * _Position_Actual_value = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_Position_Actual_value[i] = _my[i].OutParam->positionactualvalue;
				#else
					_Position_Actual_value[i] = _my.OutParam->positionactualvalue;
				#endif

				// rt_printf("MY ActualPos: %d \n\n", _Position_Actual_value[i]);
            }
		}
		break;	

		case VELOCITY_ACTUALVALUE_my:
		{
			int32_t * _Velocity_Actual_value = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_Velocity_Actual_value[i] = _my[i].OutParam->velocityactualvalue;
				#else
					_Velocity_Actual_value[i] = _my.OutParam->velocityactualvalue;
				#endif
            }
		}
		break;	

		case TORQUE_ACTUALVALUE_my:
		{
			int16_t * _Torque_Actual_value = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_Torque_Actual_value[i] = _my[i].OutParam->torqueactualvalue;
				#else
					_Torque_Actual_value[i] = _my.OutParam->torqueactualvalue;
				#endif
            }
		}
		break;	

		case ERRORCODE:
		{
			uint16_t * _errorcode = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_errorcode[i] = _my[i].OutParam->errorcode;
				#else
					_errorcode[i] = _my.OutParam->errorcode;
				#endif
            }
		}
		break;	

		case MODEofOPERATION_DISPLAY_my:
		{
			int8_t * _modeofoperation_display = static_cast<int8_t * const>(data);
            for (int i=0; i<NUM_MY; i++)
            {
				#if NUM_MY != 1
                	_modeofoperation_display[i] = _my[i].OutParam->modeofoperationdisplay;
				#else
					_modeofoperation_display[i] = _my.OutParam->modeofoperationdisplay;
				#endif
            }
		}
		break;	
		
			break;	
		default:	// Undefined Entry ID

			break;
	}
}


void EthercatMaster::servoOn(const int position, const std::string& type)
{
    if (type == "ELMO")
    {
		#if NUM_ELMO != 1
        	if (!(_elmo_gold[position].OutParam->StatusWord & (1 << STATUSWORD_OPERATION_ENABLE_BIT)))
        	{
        	    if (!(_elmo_gold[position].OutParam->StatusWord & (1 << STATUSWORD_SWITCHED_ON_BIT)))
        	    {
        	        if (!(_elmo_gold[position].OutParam->StatusWord & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT)))
        	        {
        	            if ((_elmo_gold[position].OutParam->StatusWord & (1 << STATUSWORD_FAULT_BIT)))
        	            {
        	                _elmo_gold[position].InParam->ControlWord = 0x80; // Fault reset
        	            }
        	            else
        	            {
        	                _elmo_gold[position].InParam->ControlWord = 0x06; // Shutdown
        	            }
        	        }
        	        else
        	        {
        	            _elmo_gold[position].InParam->ControlWord = 0x07; // Switch on
        	        }
        	    }
        	    else
        	    {
        	        _elmo_gold[position].InParam->ControlWord = 0x0F; // Operation enable
        	    }
        	}
        	else
        	{
        	    _elmo_gold[position].InParam->ControlWord = 0x0F; // Operation enable
        	}
			///// different /////
			ControlWord = _elmo_gold[position].InParam->ControlWord;
		#else
			if (!(_elmo_gold.OutParam->StatusWord & (1 << STATUSWORD_OPERATION_ENABLE_BIT)))
        	{
        	    if (!(_elmo_gold.OutParam->StatusWord & (1 << STATUSWORD_SWITCHED_ON_BIT)))
        	    {
        	        if (!(_elmo_gold.OutParam->StatusWord & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT)))
        	        {
        	            if ((_elmo_gold.OutParam->StatusWord & (1 << STATUSWORD_FAULT_BIT)))
        	            {
        	                _elmo_gold.InParam->ControlWord = 0x80; // Fault reset
        	            }
        	            else
        	            {
        	                _elmo_gold.InParam->ControlWord = 0x06; // Shutdown
        	            }
        	        }
        	        else
        	        {
        	            _elmo_gold.InParam->ControlWord = 0x07; // Switch on
        	        }
        	    }
        	    else
        	    {
        	        _elmo_gold.InParam->ControlWord = 0x0F; // Operation enable
        	    }
        	}
        	else
        	{
        	    _elmo_gold.InParam->ControlWord = 0x0F; // Operation enable
        	}
			///// different /////
			// ControlWord = _elmo_gold.InParam->ControlWord;
		#endif
    }
    else if (type == "MY")
    {
		#if NUM_MY != 1
        	if (!(_my[position].OutParam->statusword & (1 << STATUSWORD_OPERATION_ENABLE_BIT)))
        	{
        	    if (!(_my[position].OutParam->statusword & (1 << STATUSWORD_SWITCHED_ON_BIT)))
        	    {
        	        if (!(_my[position].OutParam->statusword & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT)))
        	        {
        	            if ((_my[position].OutParam->statusword & (1 << STATUSWORD_FAULT_BIT)))
        	            {
        	                _my[position].InParam->controlword = 0x80; // Fault reset
        	            }
        	            else
        	            {
        	                _my[position].InParam->controlword = 0x06; // Shutdown
        	            }
        	        }
        	        else
        	        {
        	            _my[position].InParam->controlword = 0x07; // Switch on
        	        }
        	    }
        	    else
        	    {
        	        _my[position].InParam->controlword = 0x0F; // Operation enable
        	    }
        	}
        	else
        	{
        	    _my[position].InParam->controlword = 0x0F; // Operation enable
        	}
			///// different /////
			controlword = _my[position].InParam->controlword;
		#else
			if (!(_my.OutParam->statusword & (1 << STATUSWORD_OPERATION_ENABLE_BIT)))
        	{
        	    if (!(_my.OutParam->statusword & (1 << STATUSWORD_SWITCHED_ON_BIT)))
        	    {
        	        if (!(_my.OutParam->statusword & (1 << STATUSWORD_READY_TO_SWITCH_ON_BIT)))
        	        {
        	            if ((_my.OutParam->statusword & (1 << STATUSWORD_FAULT_BIT)))
        	            {
        	                _my.InParam->controlword = 0x80; // Fault reset
        	            }
        	            else
        	            {
        	                _my.InParam->controlword = 0x06; // Shutdown
        	            }
        	        }
        	        else
        	        {
        	            _my.InParam->controlword = 0x07; // Switch on
        	        }
        	    }
        	    else
        	    {
        	        _my.InParam->controlword = 0x0F; // Operation enable
        	    }
        	}
        	else
        	{
        	    _my.InParam->controlword = 0x0F; // Operation enable
        	}
			///// different /////
			// controlword = _my.InParam->controlword;
		#endif
    }
}

void EthercatMaster::print_my_slave_debug_info(int slave)
{
    int nSM, j;
    uint16 ssigen;

    printf("\n=== [MYACTUATOR DEBUG] Slave %d: %s ===\n", slave, ec_slave[slave].name);
    printf(" Output size: %d bits\n", ec_slave[slave].Obits);
    printf(" Input size: %d bits\n", ec_slave[slave].Ibits);
    printf(" Configured address: 0x%04x\n", ec_slave[slave].configadr);
    printf(" Delay: %d ns\n", ec_slave[slave].pdelay);
    printf(" Has DC: %d\n", ec_slave[slave].hasdc);
    if (ec_slave[slave].hasdc) {
        printf(" DC Parent Port: %d\n", ec_slave[slave].parentport);
    }

    printf(" Active ports: %d.%d.%d.%d\n",
           (ec_slave[slave].activeports & 0x01) > 0,
           (ec_slave[slave].activeports & 0x02) > 0,
           (ec_slave[slave].activeports & 0x04) > 0,
           (ec_slave[slave].activeports & 0x08) > 0);

    for(nSM = 0 ; nSM < EC_MAXSM ; nSM++)
    {
        if(ec_slave[slave].SM[nSM].StartAddr > 0)
            printf(" SM%d: Addr=0x%04x Len=%4d Flags=0x%08x Type=%d\n",
                   nSM,
                   etohs(ec_slave[slave].SM[nSM].StartAddr),
                   etohs(ec_slave[slave].SM[nSM].SMlength),
                   etohl(ec_slave[slave].SM[nSM].SMflags),
                   ec_slave[slave].SMtype[nSM]);
    }

    for(j = 0 ; j < ec_slave[slave].FMMUunused ; j++)
    {
        printf(" FMMU%d: LogStart=0x%08x LogLen=%4d LogSB=%d LogEB=%d PhysStart=0x%04x PhysSB=%d Type=0x%02x Act=0x%02x\n",
               j,
               etohl(ec_slave[slave].FMMU[j].LogStart),
               etohs(ec_slave[slave].FMMU[j].LogLength),
               ec_slave[slave].FMMU[j].LogStartbit,
               ec_slave[slave].FMMU[j].LogEndbit,
               etohs(ec_slave[slave].FMMU[j].PhysStart),
               ec_slave[slave].FMMU[j].PhysStartBit,
               ec_slave[slave].FMMU[j].FMMUtype,
               ec_slave[slave].FMMU[j].FMMUactive);
    }

    /// === Print PDO Mapping via CoE (SDO) === ///
    if (ec_slave[slave].mbx_proto & ECT_MBXPROT_COE) {
        printf("\n--- PDO Mapping (via CoE) ---\n");

        // Output PDOs (TxPDOs: Master -> Slave)
        for (uint16 i = 0; i < 4; ++i) {
            uint16 pdo_index = 0x1600 + i;
            uint8 entries = 0;
            int size = sizeof(entries);
            int wkc = ec_SDOread(slave, pdo_index, 0x00, FALSE, &size, &entries, EC_TIMEOUTRXM);
            if (wkc <= 0 || entries == 0) break;

            printf(" TxPDO 0x%04x: %d entries\n", pdo_index, entries);
            for (uint8 k = 1; k <= entries; ++k) {
                uint32 map_entry = 0;
                size = sizeof(map_entry);
                wkc = ec_SDOread(slave, pdo_index, k, FALSE, &size, &map_entry, EC_TIMEOUTRXM);
                if (wkc > 0) {
                    uint16 idx = (map_entry >> 16) & 0xFFFF;
                    uint8 subidx = (map_entry >> 8) & 0xFF;
                    uint8 bitlen = map_entry & 0xFF;
                    printf("   - [0x%04x:%02x] %d bits\n", idx, subidx, bitlen);
                }
            }
        }

        // Input PDOs (RxPDOs: Slave -> Master)
        for (uint16 i = 0; i < 4; ++i) {
            uint16 pdo_index = 0x1A00 + i;
            uint8 entries = 0;
            int size = sizeof(entries);
            int wkc = ec_SDOread(slave, pdo_index, 0x00, FALSE, &size, &entries, EC_TIMEOUTRXM);
            if (wkc <= 0 || entries == 0) break;

            printf(" RxPDO 0x%04x: %d entries\n", pdo_index, entries);
            for (uint8 k = 1; k <= entries; ++k) {
                uint32 map_entry = 0;
                size = sizeof(map_entry);
                wkc = ec_SDOread(slave, pdo_index, k, FALSE, &size, &map_entry, EC_TIMEOUTRXM);
                if (wkc > 0) {
                    uint16 idx = (map_entry >> 16) & 0xFFFF;
                    uint8 subidx = (map_entry >> 8) & 0xFF;
                    uint8 bitlen = map_entry & 0xFF;
                    printf("   - [0x%04x:%02x] %d bits\n", idx, subidx, bitlen);
                }
            }
        }
    } else {
        printf(" No CoE protocol supported for PDO mapping.\n");
    }
}
