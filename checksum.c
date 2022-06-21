/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "checksum.h"

/*>>> add_octets: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Shreyas hemachandra
Date:		14/02/2022
Modified:	None
Desc:		This function add each bytes of the buffer (passes) of size
                (passed).
Input(1): 	Pointer to the buffer
Input(2)	Size of the buffer
Returns:    uint32_t, returns sum.
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
static uint32_t add_octets(uint8_t* buffer, uint32_t size)
{
	uint32_t sum = 0u;
	int i = 0;

	for (; i < size; i++)
	{
		sum += (uint32_t)buffer[i];
	}

	return sum;
} // eo add_octets::

/*>>> do_esComp: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Shreyas hemachandra
Date:		14/02/2022
Modified:	None
Desc:		This function does the 2's Complement of the passed uint32_t 
		value
Input(1): 	uint32_t, value
Returns:    uint32_t, returns 2's Complement.
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
static uint32_t do_2sComp(uint32_t val)
{
	val  = ~val;
	val += 1;
	return val;
} // eo do_esComp::

/*>>> cal_checksum: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Shreyas hemachandra
Date:		14/02/2022
Modified:	None
Desc:		This function calulates the checksum of the buffer (Passed) of 
		size (Passed) and returns the checksum.
Input(1): 	Pointer to the buffer
Input(2)	Size of the buffer
Returns:    uint32_t, returns the checksum of the buffer passed
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
uint32_t cal_checksum(uint8_t *buffer, uint32_t size)
{
	uint32_t checksum = 0u;

	checksum = add_octets(buffer, size);
	checksum = do_2sComp(checksum);

	return checksum;
} // eo cal_checksum::

/*>>> calnstr_checksum: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Shreyas hemachandra
Date:		14/02/2022
Modified:	None
Desc:		This function calulates the checksum of the buffer (Passed) of
		size (Passed), stores the checksum in the last 32byte address 
		and returns the checksum
Input(1): 	Pointer to the buffer
Input(2)	Size of the buffer including the checksum word
Returns:    uint32_t, returns the checksum of the buffer passed
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
uint32_t calnstr_checksum(uint8_t *buffer, uint32_t size)
{
	uint32_t  checksum = 0u;
	uint32_t* pchecksumAddr = NULL;

	checksum = cal_checksum(buffer, (size - 4));

	/* Copy checksum to last 4 buys of the buffer */
	pchecksumAddr  = (uint32_t *) & buffer[size - 4];
	*pchecksumAddr = checksum;

	return checksum;
} // eo calnstr_checksum::


/*>>> verify_checksum: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Shreyas hemachandra
Date:		14/02/2022
Modified:	None
Desc:		This function calculates the checksum of the buffer compares it to the
			last word and returns a Bool variable if both matches or not.
Input(1): 	Pointer to the buffer
Input(2)	Size of the buffer including the checksum word
Returns:    bool, 1 for checksum matched and 0 for checksum mismatch
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
bool verify_checksum(uint8_t *buffer, uint32_t size)
{
	uint32_t  calchecksum = 0u;
	uint32_t* pchecksum = NULL;

	calchecksum = add_octets(buffer, (size - 4));
	pchecksum   = (uint32_t*)&buffer[size - 4];

	/* Sum the checksum and check if its 0 */
	if (0 == (calchecksum + *pchecksum))
		return true;
	else
		return false;
} // eo verify_checksum::