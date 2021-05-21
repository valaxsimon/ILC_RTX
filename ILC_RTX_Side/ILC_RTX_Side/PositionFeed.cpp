#include "PositionFeed.hpp"
#include "../../Common/RtxLogging.hpp"


void rtvHandlerCallback(DSA_DSMAX *pUltimet, int nCycle, int nb_read, DSA_RTV_DATA *read_table[], int nb_write, DSA_RTV_DATA *write_table[], void *user)
{
	PositionFeed* rPositionFeed = (PositionFeed*)user;
	eint64 ml1;
	eint64 ml0;
	//timeStamp();
	int rtvVal;
	float rtvCurrFfwVal;
	int err;
	if (rPositionFeed->rtvInterruptNumber < rPositionFeed->gTrajectory.mLineNumber)
	{
		//if (rtvInterruptNumber<MAX_TIMESTAMPS)
		//gTimes[rtvInterruptNumber] = timeStamp();
		//start executing one point of the customer trajectory each callback

		rtvVal = rPositionFeed->gTrajectory.mPosition[rPositionFeed->rtvInterruptNumber] * rPositionFeed->isoToInc;

		if (err = dsa_write_32bit_rtv_slot(rPositionFeed->posRefSlot, rtvVal))
		{
			DSA_DIAG(err, rPositionFeed->ultimet);
			rPositionFeed->errHandler(err);
		}

		//prime
		if (USE_SLAVE == 1)
		{
			if (err = dsa_write_32bit_rtv_slot(rPositionFeed->posSlaveRefSlot, rtvVal))
			{
				DSA_DIAG(err, rPositionFeed->ultimet);
				rPositionFeed->errHandler(err);
			}
		}
		// Write the current FFW value
		rtvCurrFfwVal = rPositionFeed->gTrajectory.mCurrent[rPositionFeed->rtvInterruptNumber] * rPositionFeed->curIsoToInc;
		if(err = dsa_write_rtv_float32(rPositionFeed->curFfwRefData, rtvCurrFfwVal))
		//if (err = dsa_write_32bit_rtv_slot(rPositionFeed->curFfwRefSlot, rtvCurrFfwVal))
		{
			DSA_DIAG(err, rPositionFeed->ultimet);
			rPositionFeed->errHandler(err);
		}

		if (err = dsa_read_rtv_int64(rPositionFeed->gML0Slot, &ml0)) //extract the real trajectory values back to the PC
		{
			DSA_DIAG(err, rPositionFeed->ultimet);
			rPositionFeed->errHandler(err);
		}
		rPositionFeed->gTrajectory.mML0[rPositionFeed->rtvInterruptNumber] = ml0 / (double)rPositionFeed->isoToInc;

		if (err = dsa_read_rtv_int64(rPositionFeed->gML1Slot, &ml1)) //extract the real trajectory values back to the PC
		{
			DSA_DIAG(err, rPositionFeed->ultimet);
			rPositionFeed->errHandler(err);
		}
		rPositionFeed->gTrajectory.mML1[rPositionFeed->rtvInterruptNumber] = ml1 / (double)rPositionFeed->isoToInc;

		//update the theoretical sample rate
		rPositionFeed->gTrajectory.mTimeStamps[rPositionFeed->rtvInterruptNumber] = rPositionFeed->timeStamp();// rPositionFeed->rtvInterruptNumber * rPositionFeed->gSampleRate * 100e-6;

		//if (rtvInterruptNumber < MAX_TIMESTAMPS)
		//gTimesAfter[rtvInterruptNumber] = timeStamp();
	}
	else
	{
		if (err = dsa_stop_rtv_handler(rPositionFeed->ultimet, 0)) //stop the rtv callback as trajectory is done
		{
			DSA_DIAG(err, rPositionFeed->ultimet);
			rPositionFeed->errHandler(err);
		}

		RtPulseEvent(rPositionFeed->mTrajectoryDone); //pulse and reset only once
	}

	++rPositionFeed->rtvInterruptNumber;
}

PositionFeed::PositionFeed()
{
	QueryPerformanceFrequency((LARGE_INTEGER*)&gFrequency); //init OS timestamps system
	LPSECURITY_ATTRIBUTES ignored;
	mTrajectoryDone = RtCreateEvent(ignored, FALSE, FALSE, NULL); //unnamed event
	if (mTrajectoryDone == NULL)
		THROW("can't create trajectory done event");

	createDrives();
	destroyRTVSlots(); //free any previously assigned slots
	setupRTVSlots();
}

PositionFeed::~PositionFeed()
{
	destroyDrives();
}

void PositionFeed::writeToDrive(double* position, double* current, int n)
{
	gTrajectory.clear();
	gTrajectory.create(position, current, n);
	clearDriveErrors();
	powerOn();
	moveToStartingPos();
	startCustomerTrajectory(); //will block until trajectory is completely executed, will switch the drive in PC reference mode
	switchToAccurETReferenceMode(); //comes back to standard reference mode
}

//add here all the RTV slot that needs to be sent back to python script
void PositionFeed::readFromDrive(double* timeStamps, double* ml0, double* ml1, int* n)
{
	memcpy(timeStamps, gTrajectory.mTimeStamps, sizeof(double) * gTrajectory.mLineNumber);
	memcpy(ml0, gTrajectory.mML0, sizeof(double) * gTrajectory.mLineNumber);
	memcpy(ml1, gTrajectory.mML1, sizeof(double) * gTrajectory.mLineNumber);
	*n = gTrajectory.mLineNumber;
}

void PositionFeed::clearDriveErrors()
{
	dword M60;
	//check if the drive is in error, if yes clear them all
	if (err = dsa_get_drive_status_1_s(x, &M60, DSA_GET_CURRENT, DSA_DEF_TIMEOUT)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	bool isControllerInError = (M60 >> 10) & 0x1;
	if (isControllerInError)
	{
		if (err = dsa_reset_error_s(x, DSA_DEF_TIMEOUT)) {
			DSA_DIAG(err, x);
			errHandler(err);
		}

		if (USE_SLAVE == 1)
		{
			if (err = dsa_reset_error_s(xprime, DSA_DEF_TIMEOUT)) {
				DSA_DIAG(err, xprime);
				errHandler(err);
			}
		}
	}
}

void PositionFeed::printTimeStampsToScreen()
{
	//print timestamps to screen or file to monitor RTX callback accuracy
	FILE* ftimestamps = fopen("C:\\Users\\Administrator\\Desktop\\RtssApp2\\timestamps.txt", "w+");
	//printf("Timestamps\n");
	for (int i = 1; i < MAX_TIMESTAMPS; ++i)
	{
		//printf("%f %.9f\n", gTimes[i], gTimesAfter[i]- gTimes[i]);
		fprintf(ftimestamps, "%f %.9f\n", gTimes[i], gTimesAfter[i] - gTimes[i]);
		if ((gTimes[i] - gTimes[i - 1]) > gSampleRate * 105e-6)
		{
			printf("Error: RTX callback delayed %d %d %.9ff %.9f %.9f\n", i, i - 1, gTimes[i], gTimes[i - 1], gTimes[i] - gTimes[i - 1]);
			exit(0);
		}
	}

	fclose(ftimestamps);

	delete[] gTimes;
	delete[] gTimesAfter;
}

double PositionFeed::timeStamp()
{
	QueryPerformanceCounter((LARGE_INTEGER*)&gT2);
	return (gT2 - gT1) * 1.0 / (__int64)gFrequency;
	//printf("%f s\n", ts); //try comment this out
}

void PositionFeed::destroyRTVSlots()
{
	if (gSlotDestroyed)
		return;

	gSlotDestroyed = true; //avoid calling it twice in case of errHandler call

	if (err = dsa_free_all_transnET_slots_s(ultimet, 2000))
	{
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}
}

void PositionFeed::setupRTVSlots()
{
	//set x1 = 0 for ComET monitoring purposes
	if (err = dsa_set_register_s(x, DMD_TYP_USER, 1, 0, 0, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	//Set-up the position reference M450 RTV value
	if (err = dsa_get_32bit_rtv0_slot(ultimet, &posRefSlot))
	{
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	gSlotDestroyed = false;

	if (err = dsa_assign_slot_to_register_s(x, posRefSlot, DMD_TYP_MONITOR_INT32, 450, 0, DSA_DEF_TIMEOUT)) //M450 will be the external position
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		if (err = dsa_get_32bit_rtv0_slot(ultimet, &posSlaveRefSlot))
		{
			DSA_DIAG(err, ultimet);
			errHandler(err);
		}
	}

	gSlotDestroyed = false;

	if (USE_SLAVE == 1)
	{
		if (err = dsa_assign_slot_to_register_s(x, posSlaveRefSlot, DMD_TYP_MONITOR_INT32, 451, 0, DSA_DEF_TIMEOUT)) //M451 will be the external position
		{
			DSA_DIAG(err, x);
			errHandler(err);
		}
	}

	// Setup the current reference MF452 for current FFW value
	if(err = dsa_create_rtv_write(ultimet, DMD_INCREMENT_FLOAT32, &curFfwRefData))
	{
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}
	// Setup the current reference MF452 for current FFW value
	//if (err = dsa_get_32bit_rtv0_slot(ultimet, &curFfwRefSlot))
	//{
	//	DSA_DIAG(err, ultimet);
	//	errHandler(err);
	//}
	if(err = dsa_get_rtv_slot_of_rtv(curFfwRefData, &curFfwRefSlot))
	{
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	gSlotDestroyed = false;

	if (err = dsa_assign_slot_to_register_s(x, curFfwRefSlot, DMD_TYP_MONITOR_FLOAT32, 452, 0, DSA_DEF_TIMEOUT)) //MF452 will be the external position
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}
	// get the RTV slot number to pass it to the feedforward function RTVFFWD
	int ffwSlotnubmer;
	if(err = dsa_get_32bit_rtv_slot_nr(curFfwRefSlot, &ffwSlotnubmer)){ DSA_DIAG(err, x); errHandler(err); }
	if(err = dsa_execute_command_d_s(x,228,DMD_TYP_IMMEDIATE, ffwSlotnubmer,false,true,DSA_DEF_TIMEOUT)) { DSA_DIAG(err, x); errHandler(err); }
	if(err = dsa_set_register_float32_s(x,DMD_TYP_PPK_FLOAT32,249,0,1.0,DSA_DEF_TIMEOUT)) { DSA_DIAG(err, x); errHandler(err); } // set KF249 (position feedforward gain to 1.0f (1A = 1RTV INC))

	//MONITORINGS: Set-up the RTV for reading ML1 and ML0 position back from the AccurET through TransnET so we can compare on Matlab
	if (err = dsa_create_rtv_read(x, DMD_TYP_MONITOR_INT64, 1, 0, &gML1Slot))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_create_rtv_read(x, DMD_TYP_MONITOR_INT64, 0, 0, &gML0Slot))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}
}

void PositionFeed::startCustomerTrajectory()
{
	//reset the number of interrupt to count from the RTV handler
	rtvInterruptNumber = 0;

	//write first rtv positionvalue in M450 of axis
	int rtvVal;
	rtvVal = gTrajectory.mPosition[0] * isoToInc;

	if (err = dsa_write_32bit_rtv_slot(posRefSlot, rtvVal))
	{
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		if (err = dsa_write_32bit_rtv_slot(posSlaveRefSlot, rtvVal))
		{
			DSA_DIAG(err, ultimet);
			errHandler(err);
		}
	}

	//write first rtv curFfwRefSlot in MF452 of axis (first value is 0 for the moment)
	float rtvCurVal;
	rtvCurVal = gTrajectory.mCurrent[0] * curIsoToInc;
	if (err = dsa_write_rtv_float32(curFfwRefData, rtvVal))
	{
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	//set x1 = 1 for ComET monitoring purposes so we start the acquisition
	if (err = dsa_set_register_s(x, DMD_TYP_USER, 1, 0, 1, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	//choose between K61=4 or AccurET ITP reference mode
	switchToPCReferenceMode();

	if (err = dsa_start_delayed_rtv_handler(ultimet, 0, gSampleRate, 30, rtvHandlerCallback, 0, NULL, 0, NULL, this))
	{
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	// Enable the watchdog on user interrupt
	if (err = dsa_set_watchdog(ultimet, 100e-6 * 1000))
	{
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	RtWaitForSingleObject(mTrajectoryDone, INFINITE); //wait to be signaled from the rtx callback
	DEBUG("Wait for end of trajectory");
}

void PositionFeed::moveToStartingPos()
{
	double pos;
	if (ROTARY)
	{
		pos = gTrajectory.mPosition[0] / 360.0;//pos has to be given in turn for this function but we work in degrees!
	}
	else
	{
		pos = gTrajectory.mPosition[0];//pos has to be given in meters for this function
	}

	DEBUG("Moving to starting position %f [degrees]", pos * 360.0);

	if (err = dsa_set_target_position_s(x, 1, pos, DSA_DEF_TIMEOUT)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_new_setpoint_s(x, 1, DSA_STA_POS, DSA_DEF_TIMEOUT)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	//if (err = dsa_wait_movement_s(x, INFINITE)) { //if we do a wtm or wtp, this is only end of theorectical mvt so not good
	//	DSA_DIAG(err, x);
	//	errHandler(err);
	//}

	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 39, 0, isoToInc * unitsConversionFactor, DSA_DEF_TIMEOUT)) //window of 1udeg or 1um
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 38, 0, 2500, DSA_DEF_TIMEOUT)) //1s in window
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_wait_window_s(x, INFINITE)) { //if we do a wtm or wtp, this is only end of theorectical mvt so not good
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		//prime axis
		if (err = dsa_set_target_position_s(xprime, 1, pos, DSA_DEF_TIMEOUT)) {
			DSA_DIAG(err, x);
			errHandler(err);
		}

		if (err = dsa_new_setpoint_s(xprime, 1, DSA_STA_POS, DSA_DEF_TIMEOUT)) {
			DSA_DIAG(err, x);
			errHandler(err);
		}

		if (err = dsa_wait_movement_s(xprime, INFINITE)) { //if we do a wtm or wtp, this is only end of theorectical mvt so not good
			DSA_DIAG(err, x);
			errHandler(err);
		}
	}
}

void PositionFeed::switchToPCReferenceMode()
{
	if (ITP_REF_MODE)
		enableITPReferenceMode();
	else
		switchToExternalPositionReferenceMode();
}

void PositionFeed::switchToExternalPositionReferenceMode()
{
	//ensure K220,... registers are defined properly
	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 220, 0, 3, DSA_DEF_TIMEOUT)) // reference is a M register
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 221, 0, 450, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 222, 0, 0, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 223, 0, 0, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 224, 0, 1, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 225, 0, 1, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	//switch to external position reference mode
	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 61, 0, 4, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}
}

void PositionFeed::switchToAccurETReferenceMode()
{
	//get the actual position from the AccurET and set the KL210 target to this position to avoid a jump in theoretical position
	eint64 ML1;
	if (err = dsa_get_register_int64_s(x, DMD_TYP_MONITOR_INT64, 1, 0, &ML1, DSA_GET_CURRENT, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	//write it back to KL210 = TARGET
	if (err = dsa_set_register_int64_s(x, DMD_TYP_PPK_INT64, 210, 0, ML1, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	//set position reference to 1 = standard mode
	if (err = dsa_set_register_s(x, DMD_TYP_PPK, 61, 0, 1, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	disableITPReferenceMode();
}

void PositionFeed::enableITPReferenceMode()
{
	if (err = dsa_get_32bit_rtv_slot_nr(posRefSlot, &posRefSlotNr))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	DSA_COMMAND_PARAM param_table[3];
	param_table[0].typ = DMD_TYP_IMMEDIATE;
	param_table[0].conv = 0;
	param_table[0].val.i = 1;    // enable the position ref mode
	param_table[1].typ = DMD_TYP_IMMEDIATE;
	param_table[1].conv = 0;
	param_table[1].val.i = posRefSlotNr;
	param_table[2].typ = DMD_TYP_IMMEDIATE;
	param_table[2].conv = 0;
	param_table[2].val.i = 0;
	if (err = dsa_execute_command_x_s(x, 116, param_table, 3, 0, 0, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		if (err = dsa_get_32bit_rtv_slot_nr(posSlaveRefSlot, &posSlaveRefSlotNr))
		{
			DSA_DIAG(err, xprime);
			errHandler(err);
		}

		param_table[0].typ = DMD_TYP_IMMEDIATE;
		param_table[0].conv = 0;
		param_table[0].val.i = 1;    // enable the position ref mode
		param_table[1].typ = DMD_TYP_IMMEDIATE;
		param_table[1].conv = 0;
		param_table[1].val.i = posSlaveRefSlotNr;
		param_table[2].typ = DMD_TYP_IMMEDIATE;
		param_table[2].conv = 0;
		param_table[2].val.i = 0;
		if (err = dsa_execute_command_x_s(xprime, 116, param_table, 3, 0, 0, DSA_DEF_TIMEOUT))
		{
			DSA_DIAG(err, xprime);
			errHandler(err);
		}
	}

	itpModeEnabled = true;
}

void PositionFeed::disableITPReferenceMode()
{
	if (itpModeEnabled)
	{
		DSA_COMMAND_PARAM param_table[3];
		param_table[0].typ = DMD_TYP_IMMEDIATE;
		param_table[0].conv = 0;
		param_table[0].val.i = 0;    // disable the position ref mode
		param_table[1].typ = DMD_TYP_IMMEDIATE;
		param_table[1].conv = 0;
		param_table[1].val.i = 0;
		param_table[2].typ = DMD_TYP_IMMEDIATE;
		param_table[2].conv = 0;
		param_table[2].val.i = 0;
		if (err = dsa_execute_command_x_s(x, 116, param_table, 3, 0, 0, DSA_DEF_TIMEOUT))
		{
			DSA_DIAG(err, x);
			errHandler(err);
		}

		if (USE_SLAVE == 1)
		{
			param_table[0].typ = DMD_TYP_IMMEDIATE;
			param_table[0].conv = 0;
			param_table[0].val.i = 0;    // disable the position ref mode
			param_table[1].typ = DMD_TYP_IMMEDIATE;
			param_table[1].conv = 0;
			param_table[1].val.i = 0;
			param_table[2].typ = DMD_TYP_IMMEDIATE;
			param_table[2].conv = 0;
			param_table[2].val.i = 0;
			if (err = dsa_execute_command_x_s(xprime, 116, param_table, 3, 0, 0, DSA_DEF_TIMEOUT))
			{
				DSA_DIAG(err, xprime);
				errHandler(err);
			}
		}

		itpModeEnabled = false;
	}
}

void PositionFeed::createDrives()
{
	if (err = dsa_create_drive(&x)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		if (err = dsa_create_drive(&xprime)) {
			DSA_DIAG(err, xprime);
			errHandler(err);
		}
	}

	if (err = dsa_create_master(&ultimet)) {
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	char connectionString[256];
	sprintf(connectionString, "%s:%d", connectString, gZAxisDriveNumber);
	if (err = dsa_open_u(x, connectionString)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		sprintf(connectionString, "%s:%d", connectString, gZAxisSlaveDriveNumber);
		if (err = dsa_open_u(xprime, connectionString)) {
			DSA_DIAG(err, xprime);
			errHandler(err);
		}
	}

	sprintf(connectionString, "%s:*", connectString);
	if (err = dsa_open_u(ultimet, connectionString)) {
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	//clear any master errors
	if (err = dsa_reset_error_s(ultimet, DSA_DEF_TIMEOUT)) {
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	//clear any drive error
	clearDriveErrors();

	//get the isotoinc factor;
	if (ROTARY)
	{
		int pCod;
		if (err = dsa_get_register_int32_s(x, DMD_TYP_MONITOR, 239, 0, &pCod, DSA_GET_CURRENT, DSA_DEF_TIMEOUT))
		{
			DSA_DIAG(err, x);
			errHandler(err);
		}

		int M241;
		if (err = dsa_get_register_int32_s(x, DMD_TYP_MONITOR, 241, 0, &M241, DSA_GET_CURRENT, DSA_DEF_TIMEOUT))
		{
			DSA_DIAG(err, x);
			errHandler(err);
		}

		isoToInc = ceil(M241 * pCod / 360.0);

	}
	else
	{
		int pCod;
		if (err = dsa_get_register_int32_s(x, DMD_TYP_MONITOR, 239, 0, &pCod, DSA_GET_CURRENT, DSA_DEF_TIMEOUT))
		{
			DSA_DIAG(err, x);
			errHandler(err);
		}

		int M241;
		if (err = dsa_get_register_int32_s(x, DMD_TYP_MONITOR, 241, 0, &M241, DSA_GET_CURRENT, DSA_DEF_TIMEOUT))
		{
			DSA_DIAG(err, x);
			errHandler(err);
		}

		isoToInc = ceil(M241 / (pCod * 1e-9));
	}

	// Compute current conversion
	int M82;
	if (err = dsa_get_register_int32_s(x, DMD_TYP_MONITOR, 82, 0, &M82, DSA_GET_CURRENT, DSA_DEF_TIMEOUT));
	curIsoToInc = 32768 / (M82 / 100);
}

void PositionFeed::powerOff()
{
	if (err = dsa_power_off_s(x, 10000)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		if (err = dsa_power_off_s(xprime, 10000)) {
			DSA_DIAG(err, xprime);
			errHandler(err);
		}
	}
}

void PositionFeed::powerOn()
{
	if (err = dsa_power_on_s(x, 10000)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		if (err = dsa_power_on_s(xprime, 10000)) {
			DSA_DIAG(err, xprime);
			errHandler(err);
		}
	}

	//if homing is not done, start homing
	int M60;
	if (err = dsa_get_register_s(x, DMD_TYP_MONITOR, 60, 0, &M60, DSA_GET_CURRENT, DSA_DEF_TIMEOUT))
	{
		DSA_DIAG(err, x);
		errHandler(err);
	}
	else
	{
		if (!((M60 >> 2) & 0x1)) //homing not done yet
		{
			if (err = dsa_homing_start_s(x, 60000)) {
				DSA_DIAG(err, x);
				errHandler(err);
			}

			if (err = dsa_wait_movement_s(x, 60000)) {
				DSA_DIAG(err, x);
				errHandler(err);
			}
		}
	}

	if (USE_SLAVE == 1)
	{
		int M60;
		if (err = dsa_get_register_s(xprime, DMD_TYP_MONITOR, 60, 0, &M60, DSA_GET_CURRENT, DSA_DEF_TIMEOUT))
		{
			DSA_DIAG(err, xprime);
			errHandler(err);
		}
		else
		{
			if (!((M60 >> 2) & 0x1)) //homing not done yet
			{
				if (err = dsa_homing_start_s(xprime, 60000)) {
					DSA_DIAG(err, xprime);
					errHandler(err);
				}

				if (err = dsa_wait_movement_s(xprime, 60000)) {
					DSA_DIAG(err, xprime);
					errHandler(err);
				}
			}
		}
	}
}

void PositionFeed::destroyDrives()
{

	//if (err = dsa_power_off_s(x, 60000)) {
	//	DSA_DIAG(err, x);
	//	errHandler(err);
	//}

	if (err = dsa_close(x)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	//if (err = dsa_power_off_s(xprime, 60000)) {
	//	DSA_DIAG(err, xprime);
	//	errHandler(err);
	//}

	if (USE_SLAVE == 1)
	{
		if (err = dsa_close(xprime)) {
			DSA_DIAG(err, xprime);
			errHandler(err);
		}
	}

	if (err = dsa_close(ultimet)) {
		DSA_DIAG(err, ultimet);
		errHandler(err);
	}

	if (err = dsa_destroy(&x)) {
		DSA_DIAG(err, x);
		errHandler(err);
	}

	if (USE_SLAVE == 1)
	{
		if (err = dsa_destroy(&xprime)) {
			DSA_DIAG(err, x);
			errHandler(err);
		}
	}

	//if (err = dsa_destroy(&ultimet)) { //does not work under RTX so comment it out 
	//	DSA_DIAG(err, ultimet);
	//	errHandler(err);
	//}
}

void PositionFeed::errHandler(int& err)
{
	destroyRTVSlots();

	/* Is the drive pointer valid ? */
	if (dsa_is_valid_drive(x)) {

		/* Is the drive open ? */
		ebool open = 0;
		dsa_is_open(x, &open);
		if (open) {

			/* Stop all movements. */
			dsa_quick_stop_s(x, DSA_QS_PROGRAMMED_DEC,
				DSA_QS_BYPASS | DSA_QS_STOP_SEQUENCE, DSA_DEF_TIMEOUT);

			/* When the motor has stopped, a power off is done. */
			dsa_wait_movement_s(x, 60000);
			dsa_power_off_s(x, 10000);

			//ensure switched back to normal position mode
			switchToAccurETReferenceMode();

			/* Close the connection. */
			dsa_close(x);
		}

		/* And finally, release all resources to the OS. */
		dsa_destroy(&x);
	}

	if (USE_SLAVE == 1)
	{
		if (dsa_is_valid_drive(xprime)) {

			/* Is the drive open ? */
			ebool open = 0;
			dsa_is_open(xprime, &open);
			if (open) {

				/* Stop all movements. */
				dsa_quick_stop_s(xprime, DSA_QS_PROGRAMMED_DEC,
					DSA_QS_BYPASS | DSA_QS_STOP_SEQUENCE, DSA_DEF_TIMEOUT);

				/* When the motor has stopped, a power off is done. */
				dsa_wait_movement_s(xprime, 60000);
				dsa_power_off_s(xprime, 10000);

				//ensure switched back to normal position mode
				switchToAccurETReferenceMode();

				/* Close the connection. */
				dsa_close(xprime);
			}

			/* And finally, release all resources to the OS. */
			dsa_destroy(&xprime);
		}
	}

	/* Same operations for the ULTIMET */
	if (dsa_is_valid_master(ultimet)) {

		/* Is the drive open ? */
		ebool open = 0;
		dsa_is_open(ultimet, &open);
		if (open) {

			/* Close the connection */
			dsa_close(ultimet);
		}

		/* And finally, release all resources to the OS. */
		dsa_destroy(&ultimet);
	}

	THROW("errHandler"); //will terminate program if called from destructor which is what we want anyway
}