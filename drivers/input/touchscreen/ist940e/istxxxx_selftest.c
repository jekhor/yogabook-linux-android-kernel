#include <linux/input/ist520e.h>

#define RAWDATA_DEBUG    0

#if  RAWDATA_DEBUG
#define PRINTK_RWADATA(a,arg...)     printk(a,##arg)
#else
#define PRINTK_RWADATA(a,arg...)     do {} while (0)
#endif

#define ISTXXXX_IMAGE_DELAY	600
#define ISTCORE_RESET_IC_DELAY	500
#define HIDEEP_SELFTEST_RETRY_TIME 5
#define TEST_MODE_COMMAND_ADDR 0x0804
#define HIDEEP_IMAGE_HEAD_LEN  4
#define HIDEEP_IMAGE_CMD_4RC    0
#define HIDEEP_IMAGE_CMD_1RC    1
#define HIDEEP_IMAGE_CMD_EOP    2
#define HIDEEP_IMAGE_CMD_4RCF   3
//GIS
const unsigned short ISTXXXX_GIS_4RCF_TYPICAL_SPEC[TX_NUM] = {
1704,1883,1898,1902,1904,1910,1903,1917,1906,1918,1916,1927,1931,1952,1944,1946,1950,1968,1937,1929,1946,1950,1954,1957,1965,1951,1961,1955,1966,1969,1988,1982,1986,1968,1836
};
const unsigned short ISTXXXX_GIS_EOP_TYPICAL_SPEC[RX_NUM] = {
1500,1607,1593,1593,1586,1580,1574,1578,1570,1566,1564,1560,1562,1558,1550,1547,1547,1542,1539,1539,1533,1528,1524,1523,1516,1511,1513,1503,1502,1503,1497,1492,1492,1487,1486,1478,1470,1465,1460,1459,1455,1445,1449,1438,1435,1433,1427,1424,1419,1413,1419,1415,1426,1434,1443,1448,1458,1427
};
const unsigned char ISTXXXX_GIS_CMP_TYPICAL_SPEC[TX_NUM*RX_NUM] = {
98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,97,97,98,97,98,97,97,98,98,97,98,97,97,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,
97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,98,98,98,98,97,98,98,98,98,98,98,98,98,98,
97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,96,97,97,97,97,97,97,97,97,97,97,97,97,96,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,98,97,98,98,97,
97,97,97,96,97,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,
96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,95,96,96,96,96,96,96,96,96,95,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,97,96,96,97,97,97,97,97,97,97,97,97,
96,96,96,96,96,96,96,96,96,96,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,96,96,96,96,96,96,96,96,96,96,97,97,97,97,96,
96,96,96,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,94,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,96,96,96,96,96,96,96,96,
95,95,95,95,95,95,95,95,95,95,95,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,96,96,96,96,96,
95,95,95,95,95,95,95,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,95,
95,95,95,94,94,94,94,94,94,94,94,94,94,94,93,94,93,94,93,93,93,93,93,93,93,93,93,93,93,93,93,94,93,94,94,94,94,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,95,95,95,95,
94,94,94,94,94,94,94,94,94,94,94,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,
94,94,94,94,94,94,94,93,93,93,93,93,93,93,93,93,93,93,93,93,92,93,93,93,93,92,92,93,92,93,93,93,93,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,
94,94,94,93,94,93,93,93,93,93,93,93,93,93,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,93,93,93,93,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,94,95,94,94,
93,93,93,93,93,93,93,93,93,92,93,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,93,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,
93,93,93,93,93,93,93,92,92,92,92,92,92,92,92,92,92,92,91,91,91,92,91,91,91,92,91,91,91,92,92,92,92,92,92,92,92,92,92,92,92,92,92,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,
93,93,93,93,93,93,92,92,92,92,92,92,92,92,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,92,91,92,92,92,92,92,92,92,92,92,93,93,93,93,93,93,93,94,94,94,94,94,
93,93,93,92,92,92,92,92,92,92,92,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,92,92,92,92,92,92,92,92,92,93,93,93,93,93,93,93,94,93,
93,93,92,92,92,92,92,92,92,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,90,91,91,91,91,91,91,91,91,91,91,91,91,92,92,92,92,92,92,92,92,92,93,93,93,93,93,93,93,93,
92,92,92,92,92,92,92,91,91,91,91,91,91,91,91,91,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,91,90,90,91,91,91,91,91,91,91,91,91,91,92,92,92,92,92,92,93,93,93,93,93,93,
92,92,92,92,92,92,91,91,91,91,91,91,91,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,91,91,91,91,91,91,91,91,92,92,92,92,92,92,92,93,93,93,93,93,
92,92,92,91,91,91,91,91,91,91,91,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,89,90,89,90,90,90,90,90,90,90,90,90,90,90,90,91,91,91,91,91,91,91,91,92,92,92,92,92,92,93,93,92,
92,92,92,91,91,91,91,91,91,90,90,90,90,90,90,90,90,90,89,89,89,89,89,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,90,90,90,90,91,91,91,91,91,91,91,91,92,92,92,92,92,92,92,92,
91,92,91,91,91,91,91,91,90,90,90,90,90,90,90,90,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,90,89,90,90,90,90,90,90,90,90,90,90,91,91,91,91,91,91,91,92,92,92,92,92,92,
91,91,91,91,91,91,91,90,90,90,90,90,90,90,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,90,90,91,91,91,91,91,91,92,92,92,92,92,92,
91,91,91,91,91,91,90,90,90,90,90,90,90,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,91,91,91,91,91,91,91,92,92,92,92,92,
91,91,91,91,91,90,90,90,90,90,90,89,89,89,89,89,89,89,89,89,89,89,89,89,89,89,88,89,88,89,89,88,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,90,91,91,91,91,91,91,92,92,92,92,
91,91,91,90,90,90,90,90,90,89,89,89,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,88,89,89,89,89,89,89,89,90,90,90,90,90,90,90,91,91,91,91,91,91,92,92,92,
91,91,91,90,90,90,90,90,90,89,89,89,89,89,89,89,88,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,91,91,91,91,91,91,91,91,
90,91,91,90,90,90,90,90,89,89,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,91,91,91,91,91,91,91,
90,91,91,90,90,90,90,90,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,91,91,91,91,91,91,
90,90,90,90,90,90,90,89,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,89,89,90,90,90,90,90,90,91,91,91,91,91,91,
90,91,90,90,90,90,90,89,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,89,89,90,90,90,90,90,90,91,91,91,91,91,91,
90,90,90,90,90,90,90,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,89,90,90,90,90,90,91,91,91,91,91,91,
90,90,90,90,90,90,89,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,89,90,90,90,90,90,91,91,91,91,91,91,
91,91,90,90,90,90,90,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,87,88,88,88,88,88,88,88,88,88,88,88,89,89,89,89,89,89,90,90,90,90,90,91,91,91,91,91,91,91
};

//O-film
const unsigned short ISTXXXX_OFILM_4RCF_TYPICAL_SPEC[TX_NUM] = {
1477,1587,1600,1600,1600,1597,1601,1595,1596,1594,1591,1592,1590,1585,1584,1585,1573,1570,1571,1567,1566,1567,1560,1558,1560,1557,1558,1553,1556,1550,1548,1547,1538,1532,1375
};
const unsigned short ISTXXXX_OFILM_EOP_TYPICAL_SPEC[RX_NUM] = {
1346,1422,1405,1413,1403,1397,1397,1396,1389,1392,1387,1384,1375,1370,1365,1368,1361,1357,1358,1356,1343,1345,1340,1339,1338,1336,1333,1331,1336,1330,1334,1320,1318,1319,1315,1315,1307,1308,1315,1307,1306,1297,1297,1291,1296,1282,1286,1276,1282,1267,1271,1273,1282,1289,1295,1294,1305,1269
};
const unsigned char ISTXXXX_OFILM_CMP_TYPICAL_SPEC[TX_NUM*RX_NUM] = {
78,78,78,76,77,77,76,76,76,75,75,74,74,74,74,73,73,73,73,72,72,72,72,72,72,72,71,72,71,72,72,72,72,72,72,72,73,73,73,73,74,74,74,75,75,75,76,76,77,77,77,78,79,78,79,79,80,80,
77,77,77,75,76,76,76,75,75,75,74,74,73,73,73,73,73,72,72,72,71,72,72,71,71,71,71,71,71,71,71,71,71,72,72,72,72,73,73,73,73,74,74,74,75,75,75,76,76,77,77,78,78,78,79,79,79,79,
77,77,77,76,77,77,76,75,75,75,75,74,73,74,74,73,73,72,72,72,72,72,72,72,72,71,71,72,71,72,72,72,72,72,72,72,73,73,73,73,74,74,74,75,75,75,76,76,77,77,77,78,78,78,79,79,79,79,
77,78,77,76,77,77,76,75,75,75,75,74,74,74,74,73,73,73,72,72,72,72,72,72,72,72,71,72,72,72,72,72,72,72,73,72,73,73,73,74,74,74,74,75,75,76,76,76,77,77,78,78,79,78,79,80,79,79,
77,77,77,76,77,77,76,75,75,75,75,74,74,74,74,73,73,73,73,72,72,72,72,72,72,72,71,72,72,72,72,72,72,72,73,73,73,73,73,74,74,74,75,75,75,76,76,76,77,78,78,78,79,79,79,79,80,79,
77,78,77,76,77,77,76,75,76,75,75,75,74,74,74,73,73,73,73,73,72,73,73,72,72,72,72,72,72,72,72,72,73,73,73,73,73,74,74,74,74,75,75,75,75,76,76,77,77,78,78,78,79,79,79,80,80,80,
78,78,78,76,77,77,77,76,76,75,75,75,74,74,74,74,74,73,73,73,73,73,73,73,73,73,72,73,72,72,72,73,73,73,73,73,74,74,74,74,75,75,75,75,76,76,77,77,77,78,78,79,79,79,80,80,80,80,
78,78,78,77,78,78,77,76,76,76,76,75,75,75,75,74,74,74,74,74,73,74,73,73,73,73,73,73,73,73,73,73,73,74,74,74,74,74,74,75,75,76,76,76,76,77,77,78,78,79,79,79,80,79,80,80,80,80,
79,79,78,77,78,78,77,76,77,76,76,76,75,75,75,75,74,74,74,74,73,74,74,73,74,73,73,73,73,73,73,73,74,74,74,74,75,75,75,75,75,76,76,76,77,77,77,78,78,79,79,79,80,80,80,81,81,81,
79,79,79,77,78,78,78,77,77,77,76,76,75,75,75,75,75,74,74,74,74,74,74,74,74,74,73,74,73,74,74,74,74,74,74,74,75,75,75,75,76,76,76,76,77,77,78,78,78,79,79,80,80,80,81,81,81,81,
79,79,79,78,79,79,78,77,77,77,77,76,76,76,76,76,75,75,75,75,74,75,75,74,74,74,74,75,74,74,74,74,75,75,75,75,75,76,76,76,76,77,77,77,77,78,78,79,79,80,80,80,81,81,81,81,81,81,
80,80,80,78,79,79,79,78,78,78,77,77,77,77,77,76,76,76,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,76,76,76,76,76,77,77,77,77,78,78,78,79,79,79,80,80,81,81,81,81,82,82,82,
80,80,80,79,80,80,79,78,78,78,78,77,77,77,77,76,76,76,76,76,75,76,76,75,76,75,75,76,75,75,75,76,76,76,76,76,76,77,77,77,77,78,78,78,79,79,79,80,80,81,81,81,81,81,82,82,82,82,
81,81,80,79,80,80,79,79,79,78,78,78,77,77,77,77,77,77,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,77,76,77,77,77,77,78,78,78,78,79,79,80,80,80,81,81,81,82,82,82,83,83,83,
81,81,81,80,81,81,80,79,79,79,79,79,78,78,78,78,78,77,77,77,77,77,77,77,77,77,76,77,76,77,77,77,77,77,77,77,78,78,78,78,78,79,79,79,79,80,80,80,81,81,81,82,82,82,83,83,83,83,
82,82,82,80,81,81,81,80,80,80,80,79,79,79,79,78,78,78,78,78,77,78,78,77,77,77,77,78,77,77,77,77,78,78,78,78,78,79,79,79,79,79,80,80,80,80,81,81,81,82,82,83,83,83,83,84,84,84,
82,82,82,81,82,82,81,81,81,80,80,80,79,80,79,79,79,79,79,79,78,78,78,78,78,78,78,78,78,78,78,78,78,79,79,79,79,79,79,80,80,80,80,80,81,81,81,82,82,83,83,83,83,83,84,84,84,84,
83,83,83,82,83,82,82,81,81,81,81,81,80,80,80,80,80,79,79,79,79,79,79,79,79,79,78,79,79,79,79,79,79,79,79,79,80,80,80,80,81,81,81,81,82,82,82,82,83,83,83,84,84,84,85,85,85,85,
84,84,84,82,83,83,83,82,82,82,82,81,81,81,81,81,80,80,80,80,80,80,80,80,80,80,79,80,80,80,80,80,80,80,80,80,81,81,81,81,81,82,82,82,82,82,83,83,83,84,84,85,85,85,85,85,85,86,
84,84,84,83,84,84,83,83,83,83,83,82,82,82,82,81,81,81,81,81,80,81,81,80,81,80,80,81,80,81,81,81,81,81,81,81,81,82,82,82,82,82,83,83,83,83,84,84,84,85,85,85,85,85,86,86,86,86,
85,85,85,84,85,85,84,84,84,83,83,83,83,83,83,82,82,82,82,82,81,82,82,81,82,81,81,82,81,82,81,82,82,82,82,82,82,83,83,83,83,83,83,84,84,84,84,85,85,85,86,86,86,86,87,87,87,87,
86,86,86,85,86,85,85,84,85,84,84,84,84,84,84,83,83,83,83,83,82,83,83,82,83,82,82,83,82,82,82,83,83,83,83,83,83,83,83,84,84,84,84,84,85,85,85,85,86,86,86,87,87,87,87,88,87,88,
87,87,86,86,86,86,86,85,85,85,85,85,84,84,84,84,84,84,83,84,83,84,83,83,83,83,83,83,83,83,83,83,83,84,84,84,84,84,84,85,85,85,85,85,85,86,86,86,86,87,87,87,87,88,88,88,88,88,
88,88,87,87,87,87,87,86,86,86,86,85,85,85,85,85,85,85,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,85,85,85,85,85,85,86,86,86,86,86,86,87,87,87,88,88,88,88,88,89,89,89,89,
88,88,88,88,88,88,88,87,87,87,87,86,86,86,86,86,86,86,85,86,85,86,85,85,85,85,85,86,85,85,85,85,85,86,86,86,86,86,86,87,87,87,87,87,87,87,88,88,88,89,89,89,89,89,90,90,90,90,
89,89,89,89,89,89,88,88,88,88,88,87,87,87,87,87,87,87,86,87,86,87,86,86,86,86,86,86,86,86,86,86,87,87,87,87,87,87,87,88,88,88,88,88,88,88,89,89,89,89,90,90,90,90,90,90,90,91,
90,90,90,89,90,89,89,89,89,88,89,88,88,88,88,88,88,88,87,87,87,87,87,87,87,87,87,87,87,87,87,87,87,87,88,87,88,88,88,88,88,89,89,89,89,89,89,89,90,90,90,90,90,90,91,91,91,92,
91,91,91,90,91,91,90,90,90,90,90,89,89,89,89,89,89,89,89,89,88,89,89,89,89,88,88,89,89,88,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,90,91,91,91,91,91,92,92,92,92,92,92,92,
92,92,92,91,92,91,91,91,91,91,91,90,90,90,90,90,90,90,90,90,89,90,90,90,90,90,89,90,90,90,90,90,90,90,90,90,90,90,90,91,91,91,91,91,91,91,92,92,92,92,92,93,92,92,93,93,93,93,
93,93,92,92,92,92,92,92,92,91,92,91,91,91,91,91,91,91,91,91,91,91,91,91,91,91,90,91,91,91,91,91,91,91,91,91,91,91,91,92,92,92,92,92,92,92,93,92,93,93,93,93,93,93,94,94,94,94,
94,94,93,93,93,93,93,93,93,93,93,92,92,92,93,92,92,92,92,92,92,92,92,92,92,92,91,92,92,92,92,92,92,92,92,92,92,93,93,93,93,93,93,93,93,93,94,93,94,94,94,94,94,94,94,95,94,95,
95,95,94,94,94,94,94,94,94,93,94,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,94,93,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,95,96,
96,95,95,95,95,95,95,95,95,95,95,95,94,95,95,94,95,95,94,94,94,95,95,94,95,94,94,94,94,94,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,96,96,96,96,96,96,96,96,96,
96,96,96,96,96,96,96,96,96,96,96,96,95,96,96,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,96,96,96,96,96,96,96,96,96,96,96,96,96,97,97,97,97,97,97,97,97,97,97,97,97,
98,98,98,98,98,98,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,96,97,97,97,97,97,97,97,97,97,97,97,97,97,97,98,97,97,97,98,98,98,98,98,98,98,98,98,98,98,98,98
};


const unsigned short ISTXXXX_GIS_4RC_TYPICAL = 1211;
const unsigned short ISTXXXX_OFILM_4RC_TYPICAL = 1337;

void istxxxx_test_mode(struct ist510e *ts)
{
    unsigned char *buf;
    unsigned short *framebuf;
    unsigned short *tempframe;
	unsigned char r,t,retry;
    int ret = 0;
	unsigned int resultCNT;
	unsigned int vendor;


    buf = kmalloc(sizeof(buf)*TX_NUM*RX_NUM*2, GFP_KERNEL);
    if(buf ==NULL)
    {
        ts_log_err("can't not memmory alloc\n");
        goto exit_buf_alloc_istxxxx_test_mode;
    }
    ist_load_dwz(ts);
    vendor = ts->dwz_info.factory_id;
    printk("#####%s, vendor(Ofilm:1, GIS:6) = %d#####\n",__FUNCTION__,vendor);
    
    framebuf = kmalloc(sizeof(framebuf)*TX_NUM*RX_NUM + HIDEEP_IMAGE_HEAD_LEN *2, GFP_KERNEL);
    if(framebuf ==NULL)
    {
        ts_log_err("can't not memmory alloc\n");
        goto exit_framebuf_alloc_istxxxx_test_mode;
    }
  	

    tempframe = kmalloc(sizeof(tempframe)*TX_NUM*RX_NUM + HIDEEP_IMAGE_HEAD_LEN *2, GFP_KERNEL);
    if(tempframe ==NULL)
    {
        ts_log_err("can't not memmory alloc\n");
        goto exit_tempframe_alloc_istxxxx_test_mode;
    }

	//4RCF
	istcore_reset_ic();
	istcore_init_mode(ts);
	mdelay(ISTCORE_RESET_IC_DELAY);
	buf[0] = HIDEEP_IMAGE_CMD_4RCF;
	ret = istcore_i2c_write(ts,TEST_MODE_COMMAND_ADDR, 1, buf);
	ts_log_info("ret = %d\n", ret);
	mdelay(ISTXXXX_IMAGE_DELAY);
	retry =3;
	do{
		ret = istcore_i2c_read(ts, VR_ADDR_IMAGE, (TX_NUM*RX_NUM)*2 + HIDEEP_IMAGE_HEAD_LEN *2, (unsigned char *)framebuf);
		if(*((unsigned char *)framebuf)=='G')
		    break;
		mdelay(ISTXXXX_IMAGE_DELAY);
		retry--;
		ts_log_err("retry = %d\n", retry);
	}while(retry);
	ts_log_info("inspection\n");

	PRINTK_RWADATA("#####test 4RCF#######");
	//Inspection
	for(t=0; t<TX_NUM; t++)
	{
		resultCNT = 0;
		PRINTK_RWADATA("\n4RC spec = %d, framebuf [%d]:\n",6 == vendor?ISTXXXX_GIS_4RCF_TYPICAL_SPEC[t]:ISTXXXX_OFILM_4RCF_TYPICAL_SPEC[t],t);
		for (r=0; r<RX_NUM; r++)
		{
		#if  RAWDATA_DEBUG
		printk("%d,",framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]);

		if(r==(RX_NUM/2-1))
			printk("\n");
		#endif
			if( 6 == vendor){
			 ts_log_debug("4RC: index = %d, framebuf = %d, ISTXXXX_GIS_4RCF_TYPICAL_SPEC = %d",
						 r+t*RX_NUM,framebuf[r+t*RX_NUM], ISTXXXX_GIS_4RCF_TYPICAL_SPEC[t]);
			
				if (ISTXXXX_GIS_4RCF_TYPICAL_SPEC[t]*165 < framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]*100) // SPEC : +65% // ??
			{
				resultCNT++;
				ts_log_err("4RCF frame = %d r=%d, t=%d ", framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN], r,t);
			}
			}else if(1 == vendor){
			ts_log_debug("4RC: index = %d, framebuf = %d, ISTXXXX_OFILM_4RCF_TYPICAL_SPEC = %d",
						r+t*RX_NUM,framebuf[r+t*RX_NUM],ISTXXXX_OFILM_4RCF_TYPICAL_SPEC[t]);
			
				if (ISTXXXX_OFILM_4RCF_TYPICAL_SPEC[t]*165 < framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]*100) // SPEC : +65% // ??
			{
				resultCNT++;
				ts_log_err("4RCF frame = %d r=%d, t=%d ", framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN], r,t);
			}
			}
		}

		if (resultCNT > 0)
		{
			ts->TXshortResult[t] = 1;
			ts_log_debug("TX Channel Short = %d", t);
		}
		else
		{
			ts->TXshortResult[t] = 0;
			ts_log_debug("4RCF Pass = %d", t);
		}
	}


	//EOP
	istcore_reset_ic();
	istcore_init_mode(ts);
	mdelay(ISTCORE_RESET_IC_DELAY);
	buf[0] = 0x02;
	//buf[1] = 0x02;
	ret = istcore_i2c_write(ts,TEST_MODE_COMMAND_ADDR, 1, buf);
	ts_log_debug("ret = %d", ret);

	mdelay(ISTXXXX_IMAGE_DELAY);
	retry =3;
	do{
		ret = istcore_i2c_read(ts, VR_ADDR_IMAGE, (TX_NUM*RX_NUM)*2 + HIDEEP_IMAGE_HEAD_LEN *2, (unsigned char *)framebuf);
		if(*((unsigned char *)framebuf)=='G')
		    break;
		mdelay(ISTXXXX_IMAGE_DELAY);
		retry--;
		ts_log_err("retry = %d\n", retry);
	}while(retry);

	PRINTK_RWADATA("\n#####test EOP#######");
	//Inspection
	for(r=0; r<RX_NUM; r++)
	{
		resultCNT = 0;
		PRINTK_RWADATA("\nEOP spec = %d, framebuf [%d]:\n",6 == vendor?ISTXXXX_GIS_EOP_TYPICAL_SPEC[r]:ISTXXXX_OFILM_EOP_TYPICAL_SPEC[r],r);
		for (t=0; t<TX_NUM; t++)
		{
			#if  RAWDATA_DEBUG
			printk("%d,",framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]);
			if(t==25)
				printk("\n");
			#endif

			if( 6 == vendor){
				ts_log_debug("EOP: index = %d, framebuf = %d, ISTXXXX_GIS_EOP_TYPICAL_SPEC = %d",
							r+t*RX_NUM, framebuf[r+t*RX_NUM+ HIDEEP_IMAGE_HEAD_LEN], ISTXXXX_GIS_EOP_TYPICAL_SPEC[r]);

				if (ISTXXXX_GIS_EOP_TYPICAL_SPEC[r]*132 < framebuf[r+t*RX_NUM+ HIDEEP_IMAGE_HEAD_LEN]*100) // SPEC : +32%
				{
					ts_log_err("EOP frame = %d r=%d, t=%d", framebuf[r+t*RX_NUM+ HIDEEP_IMAGE_HEAD_LEN], r,t);
					resultCNT++;
				}
			}else if(1 == vendor){
				 ts_log_debug("EOP: index = %d, framebuf = %d, ISTXXXX_OFILM_EOP_TYPICAL_SPEC = %d",
							r+t*RX_NUM, framebuf[r+t*RX_NUM+ HIDEEP_IMAGE_HEAD_LEN], ISTXXXX_OFILM_EOP_TYPICAL_SPEC[r]);

				if (ISTXXXX_OFILM_EOP_TYPICAL_SPEC[r]*132 < framebuf[r+t*RX_NUM+ HIDEEP_IMAGE_HEAD_LEN]*100) // SPEC : +32%
				{
					ts_log_err("EOP frame = %d r=%d, t=%d ", framebuf[r+t*RX_NUM+ HIDEEP_IMAGE_HEAD_LEN], r,t);
					resultCNT++;
				}
			}
		}

		if (resultCNT > 0)
		{
			ts->RXshortResult[r] = 1;
			ts_log_err("RX Channel Short = %d", r);
		}
		else
		{
			ts->RXshortResult[r] = 0;
			ts_log_debug("EOP Pass = %d", r);
		}
	}



	//4RC
	istcore_reset_ic();
	istcore_init_mode(ts);
	mdelay(ISTCORE_RESET_IC_DELAY);
    buf[0] = 0x00;
    //buf[1] = 0x00;
    ret = istcore_i2c_write(ts,TEST_MODE_COMMAND_ADDR, 1, buf);
    ts_log_debug("ret = %d", ret);

	mdelay(ISTXXXX_IMAGE_DELAY);
	retry =3;
	do{
		ret = istcore_i2c_read(ts, VR_ADDR_IMAGE, (TX_NUM*RX_NUM)*2 + HIDEEP_IMAGE_HEAD_LEN *2, (unsigned char *)tempframe);
		if(*((unsigned char *)tempframe)=='G')
		    break;
		mdelay(ISTXXXX_IMAGE_DELAY);
		retry--;
		ts_log_err("retry = %d\n", retry);
	}while(retry);
	//1RC
	istcore_reset_ic();
	istcore_init_mode(ts);
	mdelay(ISTCORE_RESET_IC_DELAY);
	buf[0] = 0x01;
	//buf[1] = 0x01;
	ret = istcore_i2c_write(ts,TEST_MODE_COMMAND_ADDR, 1, buf);
	ts_log_debug("ret = %d", ret);

	mdelay(ISTXXXX_IMAGE_DELAY);
	retry =3;
	do{
		ret = istcore_i2c_read(ts, VR_ADDR_IMAGE, (TX_NUM*RX_NUM)*2 + HIDEEP_IMAGE_HEAD_LEN *2, (unsigned char *)framebuf);
		if(*((unsigned char *)framebuf)=='G')
		    break;
		mdelay(ISTXXXX_IMAGE_DELAY);
		retry--;
		ts_log_err("retry = %d\n", retry);
	}while(retry);

	PRINTK_RWADATA("\n#####test tempframe and framebuf#######");

	//Calculation CMP 
	for(t=0; t<TX_NUM; t++)
	{
		for (r=0; r<RX_NUM; r++)
		{
			tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN] = tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]>0?tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]:1;
			framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN] = framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]*100 / tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN];

		ts_log_debug("CMP: index = %d, framebuf(CMP) = %d, tempframe(4RC) = %d",
					 r+t*RX_NUM, framebuf[r+t*RX_NUM], tempframe[r+t*RX_NUM]);
		}
		#if  RAWDATA_DEBUG
		printk("\nCMP framebuf [%d]:\n",t);
		for (r=0; r<RX_NUM; r++){
			printk("%d,",framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]);
			if(r==(RX_NUM/2-1))
				printk("\n");
			}
		#endif
	}	

	//Inspection
	for (r=0; r<RX_NUM; r++)
	{
		if (ts->RXshortResult[r] == 0)
		{
			resultCNT = 0;
			for(t=0; t<TX_NUM; t++)
			{
				PRINTK_RWADATA("tempframe=%d\n", tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]);
				if( 6 == vendor){
					//gis
					 // SPEC : -30%
					if (ISTXXXX_GIS_4RC_TYPICAL*70 > tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]*100){
						resultCNT++;
						ts_log_err("4RC  temp = %d r=%d, t=%d ", tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN], r,t);
					}else{
						resultCNT = 0;
					}

				}else if( 1 == vendor){
					 // SPEC : -30%
					if (ISTXXXX_OFILM_4RC_TYPICAL*70 > tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]*100){
						resultCNT++;
						ts_log_err("4RC  temp = %d r=%d, t=%d ", tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN], r,t);
					}else{
						resultCNT = 0;
					}
				}
			}
			if (resultCNT > 1)
			{
				ts->RXopenResult[r] = 1;
				ts_log_err("RX Channel Open = %d", r);
			}
			else
			{
				ts->RXopenResult[r] = 0;
				ts_log_debug("RX 4RC Pass = %d", r);
			}
		}
		else
		{
			ts->RXopenResult[r] = 0;
			ts_log_debug("Skip to check 4RC RX = %d", r);
		}
		#if  RAWDATA_DEBUG
		printk("\ntempbuf [%d]:\n",r);
		for (t=0; t<TX_NUM; t++){
			printk("%d,",tempframe[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]);
			if(t==17)
				printk("\n");
			}
		#endif
	}

	PRINTK_RWADATA("\n#####test CMP#######");

	for(t=0; t<TX_NUM; t++)
	{
		PRINTK_RWADATA("\nspec = %d, framebuf [%d]:\n",6 == vendor?ISTXXXX_GIS_CMP_TYPICAL_SPEC[t]:ISTXXXX_OFILM_CMP_TYPICAL_SPEC[t],t);
		if (ts->TXshortResult[t] == 0)
		{
			resultCNT = 0;
			for (r=0; r<RX_NUM; r++)
			{
				#if  RAWDATA_DEBUG
				printk("%d,",framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]);
				if(r==(RX_NUM/2-1))
					printk("\n");
				#endif

			if( 6 == vendor){
				if ((ISTXXXX_GIS_CMP_TYPICAL_SPEC[r+t*RX_NUM]*75 > framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]*100)&&(ts->RXopenResult[r] == 0)) // SPEC : -30%
				{
					resultCNT++;
					ts_log_err("CMP  frame = %d r=%d, t=%d ", framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN], r,t);
				}
			}else if( 1 == vendor){
				if ((ISTXXXX_OFILM_CMP_TYPICAL_SPEC[r+t*RX_NUM]*75 > framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN]*100)&&(ts->RXopenResult[r] == 0)) // SPEC : -30%
				{
					resultCNT++;
					ts_log_err("CMP  frame = %d r=%d, t=%d ", framebuf[r+t*RX_NUM + HIDEEP_IMAGE_HEAD_LEN], r,t);
				}
			}
			}
			if (resultCNT > 0)
			{
				ts->TXopenResult[t] = 1;
				ts_log_debug("TX Channel Open = %d", t);
			}
			else
			{
				ts->TXopenResult[t] = 0;
				ts_log_debug("TX CMP Pass = %d", t);
			}
		}
		else
		{
			ts->TXopenResult[t] = 0;
			ts_log_debug("Skip to check CMP TX = %d", t);
		}
	}
	istcore_reset_ic();
	istcore_init_mode(ts);
	mdelay(100);
	kfree(tempframe);
	kfree(framebuf);
	kfree(buf);
	return;
	
exit_tempframe_alloc_istxxxx_test_mode:
	kfree(framebuf);
exit_framebuf_alloc_istxxxx_test_mode:
	kfree(framebuf);
exit_buf_alloc_istxxxx_test_mode:
	return;
}

