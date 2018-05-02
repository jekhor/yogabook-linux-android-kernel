
#include "hideep.h"

#define TEST_MODE_COMMAND_ADDR 0x00A0
#define TX_NUM 28
#define RX_NUM 42


const unsigned char CMP_TYPICAL_SPEC[TX_NUM*RX_NUM] = {
	85, 86, 85, 84, 85, 85, 85, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 84, 83, 83, 83, 83, 83, 83, 83, 83, 83, 84, 84, 84, 84, 84, 84, 84, 83,
	84, 84, 84, 83, 83, 83, 83, 83, 83, 83, 83, 83, 82, 83, 82, 82, 82, 82, 82, 82, 82, 82, 81, 82, 81, 81, 81, 81, 81, 81, 81, 82, 82, 81, 82, 82, 82, 82, 82, 82, 83, 81,
	82, 82, 82, 81, 81, 82, 82, 81, 81, 81, 81, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 81, 81, 80,
	80, 80, 80, 80, 80, 80, 80, 80, 79, 79, 79, 79, 78, 79, 79, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 77, 77, 77, 78, 77, 78, 78, 78, 78, 78, 78, 79, 79, 79, 79, 80, 78,
	79, 80, 79, 77, 78, 79, 78, 77, 77, 77, 77, 77, 77, 77, 77, 76, 77, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 76, 77, 77, 77, 76, 76, 76, 76, 76, 75, 75, 75, 74, 75, 75, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 74, 75, 75, 75, 75, 76, 76, 76, 75,
	76, 76, 76, 74, 74, 76, 75, 74, 74, 74, 74, 74, 73, 74, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 73, 72, 73, 72, 72, 72, 73, 73, 73, 73, 73, 73, 74, 74, 74, 74, 74, 73,
	75, 75, 74, 73, 74, 74, 74, 73, 73, 73, 72, 72, 72, 72, 72, 71, 72, 71, 71, 71, 71, 71, 71, 71, 71, 70, 70, 70, 71, 70, 71, 71, 71, 71, 72, 72, 72, 72, 73, 73, 73, 72,
	73, 74, 73, 72, 72, 73, 72, 72, 71, 71, 70, 71, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 69, 69, 69, 69, 70, 70, 70, 70, 70, 70, 70, 70, 70, 71, 72, 70,
	72, 72, 72, 70, 70, 71, 70, 70, 70, 70, 70, 69, 69, 69, 69, 67, 68, 68, 67, 67, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 69, 69, 69, 70, 70, 70, 70, 70,
	70, 71, 70, 69, 70, 70, 70, 69, 69, 68, 67, 67, 67, 67, 67, 67, 67, 67, 67, 66, 66, 66, 67, 67, 66, 66, 66, 66, 66, 66, 67, 67, 67, 67, 67, 67, 68, 68, 68, 69, 69, 67,
	70, 70, 70, 68, 68, 69, 68, 67, 67, 67, 67, 67, 66, 66, 66, 64, 65, 65, 64, 64, 64, 65, 65, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 67, 67, 67, 67, 66,
	68, 69, 68, 67, 67, 67, 67, 66, 66, 66, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 63, 64, 64, 64, 64, 64, 64, 63, 63, 64, 64, 64, 64, 64, 65, 65, 65, 65, 66, 66, 67, 65,
	67, 68, 67, 65, 66, 66, 66, 65, 64, 64, 64, 64, 63, 63, 63, 63, 63, 63, 62, 62, 62, 62, 62, 63, 62, 62, 62, 62, 62, 62, 63, 62, 63, 63, 63, 63, 64, 64, 64, 65, 65, 64,
	66, 67, 66, 64, 64, 65, 64, 64, 63, 63, 63, 63, 62, 62, 62, 61, 61, 62, 61, 60, 60, 61, 61, 60, 60, 60, 60, 60, 61, 60, 62, 61, 62, 62, 62, 62, 63, 63, 63, 64, 64, 63,
	65, 66, 65, 63, 63, 64, 64, 62, 62, 62, 62, 62, 60, 61, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 61, 61, 62, 62, 63, 63, 63, 62,
	64, 65, 64, 63, 63, 63, 62, 61, 61, 61, 60, 60, 60, 60, 60, 59, 60, 59, 59, 59, 58, 59, 59, 59, 59, 59, 59, 58, 58, 59, 60, 60, 60, 60, 60, 60, 61, 61, 61, 62, 62, 61,
	64, 64, 63, 62, 62, 62, 62, 60, 60, 60, 60, 60, 59, 59, 59, 57, 58, 58, 57, 57, 57, 57, 58, 57, 57, 57, 57, 57, 57, 57, 58, 58, 59, 58, 59, 59, 60, 60, 60, 61, 62, 60,
	63, 63, 62, 60, 61, 61, 61, 60, 60, 59, 59, 59, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 57, 57, 56, 57, 57, 57, 57, 57, 58, 58, 59, 59, 60, 60, 60, 60,
	62, 63, 62, 60, 60, 60, 60, 60, 59, 59, 58, 58, 57, 57, 57, 56, 56, 57, 56, 56, 55, 56, 56, 56, 56, 56, 56, 56, 56, 56, 57, 56, 57, 57, 57, 58, 58, 59, 59, 60, 60, 58,
	61, 62, 61, 60, 60, 60, 60, 58, 58, 58, 57, 57, 56, 57, 56, 55, 56, 56, 55, 54, 55, 55, 55, 55, 55, 54, 54, 54, 55, 54, 56, 56, 56, 56, 57, 57, 57, 57, 58, 59, 60, 58,
	60, 61, 60, 59, 59, 60, 59, 57, 57, 57, 57, 56, 56, 56, 55, 54, 55, 55, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 55, 55, 55, 55, 56, 56, 57, 57, 57, 58, 59, 57,
	60, 60, 60, 58, 58, 59, 58, 57, 57, 57, 56, 56, 55, 55, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 53, 54, 54, 54, 54, 54, 54, 55, 56, 56, 57, 57, 58, 58, 57,
	60, 60, 60, 57, 57, 58, 57, 56, 56, 56, 56, 55, 54, 54, 54, 53, 54, 54, 53, 53, 53, 53, 53, 54, 53, 53, 53, 53, 53, 53, 54, 54, 54, 54, 55, 55, 56, 56, 56, 57, 57, 56,
	60, 60, 60, 57, 57, 58, 57, 56, 56, 56, 55, 55, 54, 54, 54, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 55, 55, 56, 56, 57, 57, 56,
	59, 60, 59, 57, 57, 57, 57, 56, 56, 55, 55, 54, 54, 54, 53, 53, 53, 53, 53, 52, 52, 52, 53, 53, 53, 53, 53, 52, 52, 53, 53, 53, 54, 53, 54, 54, 55, 55, 56, 57, 57, 56,
	59, 60, 58, 57, 57, 57, 57, 55, 55, 55, 54, 54, 53, 54, 53, 52, 53, 53, 52, 52, 52, 52, 52, 53, 53, 53, 52, 52, 52, 53, 53, 53, 54, 53, 54, 54, 55, 55, 56, 56, 57, 55,
	59, 59, 58, 56, 56, 56, 56, 54, 54, 54, 54, 53, 53, 53, 52, 51, 52, 52, 51, 50, 50, 51, 51, 52, 51, 51, 50, 51, 51, 51, 52, 52, 52, 52, 53, 53, 54, 54, 55, 56, 56, 55,

};

const unsigned short HIDEEP_4RCF_TYPICAL_SPEC[TX_NUM] = {
	1081, 1281, 1317, 1328, 1335, 1340, 1347, 1354, 1360, 1366, 1373, 1379, 1385, 1384, 1386, 1400, 1406, 1410, 1413, 1416, 1418, 1421, 1421, 1419, 1416, 1406, 1373, 1191,
};

const unsigned short HIDEEP_EOP_TYPICAL_SPEC[RX_NUM] = {
	913, 943, 937, 914, 914, 926, 914, 914, 909, 909, 911, 916, 913, 925, 913, 926, 930, 932, 938, 938, 943, 945, 949, 950, 953, 957, 961, 962, 967, 973, 974, 976, 997, 986, 989, 989, 1001, 1001, 1001, 1007, 1019, 952
};

const unsigned short HIDEEP_4RC_TYPICAL = 837;

void hideep_test_mode(struct hideep_data *ts)
{
    unsigned char *buf;
    unsigned short *framebuf;
    unsigned short *tempframe;
	unsigned char r,t;
    int ret = 0;
	unsigned int resultCNT;


    buf = kmalloc(sizeof(buf)*TX_NUM*RX_NUM*2, GFP_KERNEL);
  if(buf ==NULL)
    {
        dbg_err("can't not memmory alloc");
        goto exit_buf_alloc_hideep_test_mode;
    }
    
    framebuf = kmalloc(sizeof(framebuf)*TX_NUM*RX_NUM, GFP_KERNEL);
  if(framebuf ==NULL)
    {
        dbg_err("can't not memmory alloc");
        goto exit_framebuf_alloc_hideep_test_mode;
    }
  	

    tempframe = kmalloc(sizeof(tempframe)*TX_NUM*RX_NUM, GFP_KERNEL);
  if(tempframe ==NULL)
    {
        dbg_err("can't not memmory alloc");
        goto exit_tempframe_alloc_hideep_test_mode;
    }
  	

	//4RCF
	hideep_reset();
	mdelay(100);
	buf[0] = 0x01;
	buf[1] = 0x03;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 2, buf);
	dbg_log("ret = %d", ret);

	mdelay(500);
	ret = hideep_i2c_read(ts->client, 0x7000, 1, buf);

	buf[0] = 0x02;
	buf[1] = 0; //TX_NUM;
	buf[2] = 0; //RX_NUM;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 3, buf);
	dbg_log("ret = %d", ret);
	ret = hideep_i2c_read(ts->client, 0x7000, (TX_NUM*RX_NUM)*2, (unsigned char *)framebuf);

	//Inspection
	for(t=0; t<TX_NUM; t++)
	{
		resultCNT = 0;
		for (r=0; r<RX_NUM; r++)
		{
            dbg_log("4RC: index = %d, framebuf = %d, HIDEEP_4RCF_TYPICAL_SPEC = %d", 
                r+t*RX_NUM,
                framebuf[r+t*RX_NUM],
                HIDEEP_4RCF_TYPICAL_SPEC[t]);
			
			if (HIDEEP_4RCF_TYPICAL_SPEC[t]*13 < framebuf[r+t*RX_NUM]*10) // SPEC : +30% // ??
			{
				resultCNT++;
			}
		}

		if (resultCNT > 0)
		{
			ts->TXshortResult[t] = 1;
			dbg_log("TX Channel Short = %d", t);
		}
		else
		{
			ts->TXshortResult[t] = 0;
			dbg_log("4RCF Pass = %d", t);
		}
	}


	//EOP
	hideep_reset();
	mdelay(100);
	buf[0] = 0x01;
	buf[1] = 0x02;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 2, buf);
	dbg_log("ret = %d", ret);

	mdelay(500);
	ret = hideep_i2c_read(ts->client, 0x7000, 1, buf);

	buf[0] = 0x02;
	buf[1] = 0; //TX_NUM;
	buf[2] = 0; //RX_NUM;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 3, buf);
	dbg_log("ret = %d", ret);
	ret = hideep_i2c_read(ts->client, 0x7000, (TX_NUM*RX_NUM)*2, (unsigned char *)framebuf);

	//Inspection
	for(r=0; r<RX_NUM; r++)
	{
		resultCNT = 0;
		for (t=0; t<TX_NUM; t++)
		{
            dbg_log("EOP: index = %d, framebuf = %d, HIDEEP_EOP_TYPICAL_SPEC = %d", 
                r+t*RX_NUM,
                framebuf[r+t*RX_NUM],
                HIDEEP_EOP_TYPICAL_SPEC[r]);

			if (HIDEEP_EOP_TYPICAL_SPEC[r]*12 < framebuf[r+t*RX_NUM]*10) // SPEC : +20%
			{
				resultCNT++;
			}
		}

		if (resultCNT > 0)
		{
			ts->RXshortResult[r] = 1;
			dbg_log("RX Channel Short = %d", r);
		}
		else
		{
			ts->RXshortResult[r] = 0;
			dbg_log("EOP Pass = %d", r);
		}
	}



	//4RC
	hideep_reset();
	mdelay(100);
    buf[0] = 0x01;
    buf[1] = 0x00;
    ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 2, buf);
    dbg_log("ret = %d", ret);

	mdelay(500);
	ret = hideep_i2c_read(ts->client, 0x7000, 1, buf);

	buf[0] = 0x02;
	buf[1] = 0; //TX_NUM;
	buf[2] = 0; //RX_NUM;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 3, buf);
	dbg_log("ret = %d", ret);

	ret = hideep_i2c_read(ts->client, 0x7000, (TX_NUM*RX_NUM)*2, (unsigned char *)tempframe);

	//1RC
	hideep_reset();
	mdelay(100);
	buf[0] = 0x01;
	buf[1] = 0x01;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 2, buf);
	dbg_log("ret = %d", ret);

	mdelay(500);
	ret = hideep_i2c_read(ts->client, 0x7000, 1, buf);

	buf[0] = 0x02;
	buf[1] = 0; //TX_NUM;
	buf[2] = 0; //RX_NUM;
	ret = hideep_i2c_write(ts->client,TEST_MODE_COMMAND_ADDR, 3, buf);
	dbg_log("ret = %d", ret);
	ret = hideep_i2c_read(ts->client, 0x7000, (TX_NUM*RX_NUM)*2, (unsigned char *)framebuf);

	//Calculation CMP 
	for(t=0; t<TX_NUM; t++)
	{
		for (r=0; r<RX_NUM; r++)
		{
			framebuf[r+t*RX_NUM] = framebuf[r+t*RX_NUM]*100 / tempframe[r+t*RX_NUM];

            dbg_log("CMP: index = %d, framebuf(CMP) = %d, tempframe(4RC) = %d", 
                r+t*RX_NUM,
                framebuf[r+t*RX_NUM],
                tempframe[r+t*RX_NUM]);
		}
	}	

	//Inspection
	for (r=0; r<RX_NUM; r++)
	{
		if (ts->RXshortResult[r] == 0)
		{
			resultCNT = 0;
			for(t=0; t<TX_NUM; t++)
			{
				if (HIDEEP_4RC_TYPICAL*7 > tempframe[r+t*RX_NUM]*10) // SPEC : -30%
				{
					resultCNT++;
				}
				else
				{
					resultCNT = 0;
				}
			}
			if (resultCNT > 1)
			{
				ts->RXopenResult[r] = 1;
				dbg_log("RX Channel Open = %d", r);
			}
			else
			{
				ts->RXopenResult[r] = 0;
				dbg_log("RX 4RC Pass = %d", r);
			}
		}
		else
		{
			ts->RXopenResult[r] = 0;
			dbg_log("Skip to check 4RC RX = %d", r);
		}
	}
	

	for(t=0; t<TX_NUM; t++)
	{
		if (ts->TXshortResult[t] == 0)
		{
			resultCNT = 0;
			for (r=0; r<RX_NUM; r++)
			{
				if ((CMP_TYPICAL_SPEC[r+t*RX_NUM]*8 > framebuf[r+t*RX_NUM]*10)&&(ts->RXopenResult[r] == 0)) // SPEC : -20%
				{
					resultCNT++;
				}
			}

			if (resultCNT > 0)
			{
				ts->TXopenResult[t] = 1;
				dbg_log("TX Channel Open = %d", t);
			}
			else
			{
				ts->TXopenResult[t] = 0;
				dbg_log("TX CMP Pass = %d", t);
			}
		}
		else
		{
			ts->TXopenResult[t] = 0;
			dbg_log("Skip to check CMP TX = %d", t);
		}
	}
	hideep_reset();
	mdelay(100);
	kfree(tempframe);
	kfree(framebuf);
	kfree(buf);
	return;
	
exit_tempframe_alloc_hideep_test_mode:
	kfree(framebuf);
exit_framebuf_alloc_hideep_test_mode:
	kfree(framebuf);
exit_buf_alloc_hideep_test_mode:
	return;
}


