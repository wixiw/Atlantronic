
/* File generated by gen_cfile.py. Should not be modified. */

#include "ObjDict.h"

/**************************************************************************/
/* Declaration of mapped variables                                        */
/**************************************************************************/
UNS8 Read_Inputs_8_Bit[] =		/* Mapped at index 0x6000, subindex 0x01 - 0x01 */
  {
    0x0	/* 0 */
  };
UNS8 Polarity_Input_8_Bit[] =		/* Mapped at index 0x6002, subindex 0x01 - 0x01 */
  {
    0x0	/* 0 */
  };
UNS8 Write_Outputs_8_Bit[] =		/* Mapped at index 0x6200, subindex 0x01 - 0x01 */
  {
    0x0	/* 0 */
  };
UNS8 Change_Polarity_Outputs_8_Bit[] =		/* Mapped at index 0x6202, subindex 0x01 - 0x01 */
  {
    0x0	/* 0 */
  };
UNS8 Error_Mode_Outputs_8_Bit[] =		/* Mapped at index 0x6206, subindex 0x01 - 0x01 */
  {
    0x0	/* 0 */
  };
UNS8 Error_Value_Outputs_8_Bit[] =		/* Mapped at index 0x6207, subindex 0x01 - 0x01 */
  {
    0x0	/* 0 */
  };
INTEGER16 Read_Analogue_Input_16_Bit[] =		/* Mapped at index 0x6401, subindex 0x01 - 0x08 */
  {
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0	/* 0 */
  };
INTEGER16 Write_Analogue_Output_16_Bit[] =		/* Mapped at index 0x6411, subindex 0x01 - 0x04 */
  {
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0	/* 0 */
  };
UNS8 Analogue_Input_Global_Interrupt_Enable = 0x0;		/* Mapped at index 0x6423, subindex 0x00 */
UNS32 Analogue_Input_Interrupt_Delta_Unsigned[] =		/* Mapped at index 0x6426, subindex 0x01 - 0x08 */
  {
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0	/* 0 */
  };
UNS8 Analogue_Output_Error_Mode[] =		/* Mapped at index 0x6443, subindex 0x01 - 0x04 */
  {
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0	/* 0 */
  };
INTEGER32 Analogue_Output_Error_Value_Integer[] =		/* Mapped at index 0x6444, subindex 0x01 - 0x04 */
  {
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0	/* 0 */
  };

/**************************************************************************/
/* Declaration of value range types                                       */
/**************************************************************************/

#define valueRange_EMC 0x9F /* Type for index 0x1003 subindex 0x00 (only set of value 0 is possible) */
UNS32 ObjDict_valueRangeTest (UNS8 typeValue, void * value)
{
  switch (typeValue) {
    case valueRange_EMC:
      if (*(UNS8*)value != (UNS8)0) return OD_VALUE_RANGE_EXCEEDED;
      break;
  }
  return 0;
}

/**************************************************************************/
/* The node id                                                            */
/**************************************************************************/
/* node_id default value.*/
UNS8 ObjDict_bDeviceNodeId = 0x00;

/**************************************************************************/
/* Array of message processing information */

const UNS8 ObjDict_iam_a_slave = 1;

TIMER_HANDLE ObjDict_heartBeatTimers[3] = {TIMER_NONE,TIMER_NONE,TIMER_NONE};

/*
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

                               OBJECT DICTIONARY

$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
*/

/* index 0x1000 :   Device Type. */
                    UNS32 ObjDict_obj1000 = 0xF0191;	/* 983441 */
                    subindex ObjDict_Index1000[] = 
                     {
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1000 }
                     };

/* index 0x1001 :   Error Register. */
                    UNS8 ObjDict_obj1001 = 0x0;	/* 0 */
                    subindex ObjDict_Index1001[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_obj1001 }
                     };

/* index 0x1002 :   Manufacturer Status Register. */
                    UNS32 ObjDict_obj1002 = 0x0;	/* 0 */
                    subindex ObjDict_Index1002[] = 
                     {
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1002 }
                     };

/* index 0x1003 :   Pre-defined Error Field */
                    UNS8 ObjDict_highestSubIndex_obj1003 = 0; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1003[] = 
                    {
                      0x0	/* 0 */
                    };
                    ODCallback_t ObjDict_Index1003_callbacks[] = 
                     {
                       NULL,
                       NULL,
                     };
                    subindex ObjDict_Index1003[] = 
                     {
                       { RW, valueRange_EMC, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1003 },
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1003[0] }
                     };

/* index 0x1005 :   SYNC COB ID. */
                    UNS32 ObjDict_obj1005 = 0x80;	/* 128 */
                    ODCallback_t ObjDict_Index1005_callbacks[] = 
                     {
                       NULL,
                     };
                    subindex ObjDict_Index1005[] = 
                     {
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1005 }
                     };

/* index 0x1006 :   Communication / Cycle Period */
                    UNS32 ObjDict_obj1006 = 0x0;   /* 0 */

/* index 0x1008 :   Manufacturer Device Name. */
                    INTEGER8 ObjDict_obj1008[10] = "";
                    subindex ObjDict_Index1008[] = 
                     {
                       { RO, visible_string, 10, (void*)&ObjDict_obj1008 }
                     };

/* index 0x1009 :   Manufacturer Hardware Version. */
                    INTEGER8 ObjDict_obj1009[10] = "";
                    subindex ObjDict_Index1009[] = 
                     {
                       { RO, visible_string, 10, (void*)&ObjDict_obj1009 }
                     };

/* index 0x100A :   Manufacturer Software Version. */
                    INTEGER8 ObjDict_obj100A[10] = "";
                    subindex ObjDict_Index100A[] = 
                     {
                       { RO, visible_string, 10, (void*)&ObjDict_obj100A }
                     };

/* index 0x100C :   Guard Time. */
                    UNS16 ObjDict_obj100C = 0x0;	/* 0 */
                    subindex ObjDict_Index100C[] = 
                     {
                       { RW, uint16, sizeof (UNS16), (void*)&ObjDict_obj100C }
                     };

/* index 0x100D :   Life Time Factor. */
                    UNS8 ObjDict_obj100D = 0x0;	/* 0 */
                    subindex ObjDict_Index100D[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj100D }
                     };

/* index 0x1010 :   Store parameters. */
                    UNS8 ObjDict_highestSubIndex_obj1010 = 1; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1010_Save_All_Parameters = 0x0;	/* 0 */
                    subindex ObjDict_Index1010[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1010 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1010_Save_All_Parameters }
                     };

/* index 0x1011 :   Restore Default Parameters. */
                    UNS8 ObjDict_highestSubIndex_obj1011 = 1; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1011_Restore_All_Default_Parameters = 0x0;	/* 0 */
                    subindex ObjDict_Index1011[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1011 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1011_Restore_All_Default_Parameters }
                     };

/* index 0x1014 :   Emergency COB ID */
                    UNS32 ObjDict_obj1014 = 0x80 + 0x00;   /* 128 + NodeID */

/* index 0x1016 :   Consumer Heartbeat Time. */
                    UNS8 ObjDict_highestSubIndex_obj1016 = 3; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1016[] = 
                    {
                      0x0,	/* 0 */
                      0x0,	/* 0 */
                      0x0	/* 0 */
                    };
                    subindex ObjDict_Index1016[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1016 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1016[0] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1016[1] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1016[2] }
                     };

/* index 0x1017 :   Producer Heartbeat Time. */
                    UNS16 ObjDict_obj1017 = 0x0;	/* 0 */
                    ODCallback_t ObjDict_Index1017_callbacks[] = 
                     {
                       NULL,
                     };
                    subindex ObjDict_Index1017[] = 
                     {
                       { RW, uint16, sizeof (UNS16), (void*)&ObjDict_obj1017 }
                     };

/* index 0x1018 :   Identity. */
                    UNS8 ObjDict_highestSubIndex_obj1018 = 3; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1018_Vendor_ID = 0x175;	/* 373 */
                    UNS32 ObjDict_obj1018_Product_Code = 0x100000;	/* 1048576 */
                    UNS32 ObjDict_obj1018_Revision_Number = 0x10001;	/* 65537 */
                    subindex ObjDict_Index1018[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1018 },
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1018_Vendor_ID },
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1018_Product_Code },
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1018_Revision_Number }
                     };

/* index 0x1020 :   Verify Configuration. */
                    UNS8 ObjDict_highestSubIndex_obj1020 = 2; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1020_Configuration_Date = 0x0;	/* 0 */
                    UNS32 ObjDict_obj1020_Configuration_Time = 0x0;	/* 0 */
                    subindex ObjDict_Index1020[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1020 },
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1020_Configuration_Date },
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1020_Configuration_Time }
                     };

/* index 0x1200 :   Server SDO Parameter. */
                    UNS8 ObjDict_highestSubIndex_obj1200 = 2; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1200_COB_ID_Client_to_Server_Receive_SDO = 0x600;	/* 1536 */
                    UNS32 ObjDict_obj1200_COB_ID_Server_to_Client_Transmit_SDO = 0x580;	/* 1408 */
                    subindex ObjDict_Index1200[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1200 },
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1200_COB_ID_Client_to_Server_Receive_SDO },
                       { RO, uint32, sizeof (UNS32), (void*)&ObjDict_obj1200_COB_ID_Server_to_Client_Transmit_SDO }
                     };

/* index 0x1400 :   Receive PDO 1 Parameter. */
                    UNS8 ObjDict_highestSubIndex_obj1400 = 2; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1400_COB_ID_used_by_PDO = 0x200;	/* 512 */
                    UNS8 ObjDict_obj1400_Transmission_Type = 0x1;	/* 1 */
                    subindex ObjDict_Index1400[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1400 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1400_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj1400_Transmission_Type }
                     };

/* index 0x1401 :   Receive PDO 2 Parameter. */
                    UNS8 ObjDict_highestSubIndex_obj1401 = 2; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1401_COB_ID_used_by_PDO = 0x300;	/* 768 */
                    UNS8 ObjDict_obj1401_Transmission_Type = 0x1;	/* 1 */
                    subindex ObjDict_Index1401[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1401 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1401_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj1401_Transmission_Type }
                     };

/* index 0x1600 :   Receive PDO 1 Mapping. */
                    UNS8 ObjDict_highestSubIndex_obj1600 = 1; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1600[] = 
                    {
                      0x62000108	/* 1644167432 */
                    };
                    subindex ObjDict_Index1600[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1600 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1600[0] }
                     };

/* index 0x1601 :   Receive PDO 2 Mapping. */
                    UNS8 ObjDict_highestSubIndex_obj1601 = 4; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1601[] = 
                    {
                      0x64110110,	/* 1678835984 */
                      0x64110210,	/* 1678836240 */
                      0x64110310,	/* 1678836496 */
                      0x64110410	/* 1678836752 */
                    };
                    subindex ObjDict_Index1601[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1601 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1601[0] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1601[1] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1601[2] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1601[3] }
                     };

/* index 0x1800 :   Transmit PDO 1 Parameter. */
                    UNS8 ObjDict_highestSubIndex_obj1800 = 5; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1800_COB_ID_used_by_PDO = 0x180;	/* 384 */
                    UNS8 ObjDict_obj1800_Transmission_Type = 0x1;	/* 1 */
                    UNS16 ObjDict_obj1800_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 ObjDict_obj1800_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 ObjDict_obj1800_Event_Timer = 0x0;	/* 0 */
                    ODCallback_t ObjDict_Index1800_callbacks[] = 
                     {
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                     };
                    subindex ObjDict_Index1800[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1800 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1800_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj1800_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&ObjDict_obj1800_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj1800_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&ObjDict_obj1800_Event_Timer }
                     };

/* index 0x1801 :   Transmit PDO 2 Parameter. */
                    UNS8 ObjDict_highestSubIndex_obj1801 = 5; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1801_COB_ID_used_by_PDO = 0x280;	/* 640 */
                    UNS8 ObjDict_obj1801_Transmission_Type = 0x1;	/* 1 */
                    UNS16 ObjDict_obj1801_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 ObjDict_obj1801_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 ObjDict_obj1801_Event_Timer = 0x0;	/* 0 */
                    ODCallback_t ObjDict_Index1801_callbacks[] = 
                     {
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                     };
                    subindex ObjDict_Index1801[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1801 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1801_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj1801_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&ObjDict_obj1801_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj1801_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&ObjDict_obj1801_Event_Timer }
                     };

/* index 0x1802 :   Transmit PDO 3 Parameter. */
                    UNS8 ObjDict_highestSubIndex_obj1802 = 5; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1802_COB_ID_used_by_PDO = 0x380;	/* 896 */
                    UNS8 ObjDict_obj1802_Transmission_Type = 0x1;	/* 1 */
                    UNS16 ObjDict_obj1802_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 ObjDict_obj1802_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 ObjDict_obj1802_Event_Timer = 0x0;	/* 0 */
                    ODCallback_t ObjDict_Index1802_callbacks[] = 
                     {
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                       NULL,
                     };
                    subindex ObjDict_Index1802[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1802 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1802_COB_ID_used_by_PDO },
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj1802_Transmission_Type },
                       { RW, uint16, sizeof (UNS16), (void*)&ObjDict_obj1802_Inhibit_Time },
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_obj1802_Compatibility_Entry },
                       { RW, uint16, sizeof (UNS16), (void*)&ObjDict_obj1802_Event_Timer }
                     };

/* index 0x1A00 :   Transmit PDO 1 Mapping. */
                    UNS8 ObjDict_highestSubIndex_obj1A00 = 1; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1A00[] = 
                    {
                      0x60000108	/* 1610613000 */
                    };
                    subindex ObjDict_Index1A00[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1A00 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A00[0] }
                     };

/* index 0x1A01 :   Transmit PDO 2 Mapping. */
                    UNS8 ObjDict_highestSubIndex_obj1A01 = 4; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1A01[] = 
                    {
                      0x64010110,	/* 1677787408 */
                      0x64010210,	/* 1677787664 */
                      0x64010310,	/* 1677787920 */
                      0x64010410	/* 1677788176 */
                    };
                    subindex ObjDict_Index1A01[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1A01 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A01[0] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A01[1] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A01[2] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A01[3] }
                     };

/* index 0x1A02 :   Transmit PDO 3 Mapping. */
                    UNS8 ObjDict_highestSubIndex_obj1A02 = 4; /* number of subindex - 1*/
                    UNS32 ObjDict_obj1A02[] = 
                    {
                      0x64010510,	/* 1677788432 */
                      0x64010610,	/* 1677788688 */
                      0x64010710,	/* 1677788944 */
                      0x64010810	/* 1677789200 */
                    };
                    subindex ObjDict_Index1A02[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj1A02 },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A02[0] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A02[1] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A02[2] },
                       { RW, uint32, sizeof (UNS32), (void*)&ObjDict_obj1A02[3] }
                     };

/* index 0x6000 :   Mapped variable Read Inputs 8 Bit */
                    UNS8 ObjDict_highestSubIndex_obj6000 = 1; /* number of subindex - 1*/
                    subindex ObjDict_Index6000[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6000 },
                       { RO, uint8, sizeof (UNS8), (void*)&Read_Inputs_8_Bit[0] }
                     };

/* index 0x6002 :   Mapped variable Polarity Input 8 Bit */
                    UNS8 ObjDict_highestSubIndex_obj6002 = 1; /* number of subindex - 1*/
                    subindex ObjDict_Index6002[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6002 },
                       { RW, uint8, sizeof (UNS8), (void*)&Polarity_Input_8_Bit[0] }
                     };

/* index 0x6200 :   Mapped variable Write Outputs 8 Bit */
                    UNS8 ObjDict_highestSubIndex_obj6200 = 1; /* number of subindex - 1*/
                    subindex ObjDict_Index6200[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6200 },
                       { RW, uint8, sizeof (UNS8), (void*)&Write_Outputs_8_Bit[0] }
                     };

/* index 0x6202 :   Mapped variable Change Polarity Outputs 8 Bit */
                    UNS8 ObjDict_highestSubIndex_obj6202 = 1; /* number of subindex - 1*/
                    subindex ObjDict_Index6202[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6202 },
                       { RW, uint8, sizeof (UNS8), (void*)&Change_Polarity_Outputs_8_Bit[0] }
                     };

/* index 0x6206 :   Mapped variable Error Mode Outputs 8 Bit */
                    UNS8 ObjDict_highestSubIndex_obj6206 = 1; /* number of subindex - 1*/
                    subindex ObjDict_Index6206[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6206 },
                       { RW, uint8, sizeof (UNS8), (void*)&Error_Mode_Outputs_8_Bit[0] }
                     };

/* index 0x6207 :   Mapped variable Error Value Outputs 8 Bit */
                    UNS8 ObjDict_highestSubIndex_obj6207 = 1; /* number of subindex - 1*/
                    subindex ObjDict_Index6207[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6207 },
                       { RW, uint8, sizeof (UNS8), (void*)&Error_Value_Outputs_8_Bit[0] }
                     };

/* index 0x6401 :   Mapped variable Read Analogue Input 16 Bit */
                    UNS8 ObjDict_highestSubIndex_obj6401 = 8; /* number of subindex - 1*/
                    subindex ObjDict_Index6401[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6401 },
                       { RO, int16, sizeof (INTEGER16), (void*)&Read_Analogue_Input_16_Bit[0] },
                       { RO, int16, sizeof (INTEGER16), (void*)&Read_Analogue_Input_16_Bit[1] },
                       { RO, int16, sizeof (INTEGER16), (void*)&Read_Analogue_Input_16_Bit[2] },
                       { RO, int16, sizeof (INTEGER16), (void*)&Read_Analogue_Input_16_Bit[3] },
                       { RO, int16, sizeof (INTEGER16), (void*)&Read_Analogue_Input_16_Bit[4] },
                       { RO, int16, sizeof (INTEGER16), (void*)&Read_Analogue_Input_16_Bit[5] },
                       { RO, int16, sizeof (INTEGER16), (void*)&Read_Analogue_Input_16_Bit[6] },
                       { RO, int16, sizeof (INTEGER16), (void*)&Read_Analogue_Input_16_Bit[7] }
                     };

/* index 0x6411 :   Mapped variable Write Analogue Output 16 Bit */
                    UNS8 ObjDict_highestSubIndex_obj6411 = 4; /* number of subindex - 1*/
                    subindex ObjDict_Index6411[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6411 },
                       { RW, int16, sizeof (INTEGER16), (void*)&Write_Analogue_Output_16_Bit[0] },
                       { RW, int16, sizeof (INTEGER16), (void*)&Write_Analogue_Output_16_Bit[1] },
                       { RW, int16, sizeof (INTEGER16), (void*)&Write_Analogue_Output_16_Bit[2] },
                       { RW, int16, sizeof (INTEGER16), (void*)&Write_Analogue_Output_16_Bit[3] }
                     };

/* index 0x6423 :   Mapped variable Analogue Input Global Interrupt Enable */
                    subindex ObjDict_Index6423[] = 
                     {
                       { RW, boolean, sizeof (UNS8), (void*)&Analogue_Input_Global_Interrupt_Enable }
                     };

/* index 0x6426 :   Mapped variable Analogue Input Interrupt Delta Unsigned */
                    UNS8 ObjDict_highestSubIndex_obj6426 = 8; /* number of subindex - 1*/
                    subindex ObjDict_Index6426[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6426 },
                       { RW, uint32, sizeof (UNS32), (void*)&Analogue_Input_Interrupt_Delta_Unsigned[0] },
                       { RW, uint32, sizeof (UNS32), (void*)&Analogue_Input_Interrupt_Delta_Unsigned[1] },
                       { RW, uint32, sizeof (UNS32), (void*)&Analogue_Input_Interrupt_Delta_Unsigned[2] },
                       { RW, uint32, sizeof (UNS32), (void*)&Analogue_Input_Interrupt_Delta_Unsigned[3] },
                       { RW, uint32, sizeof (UNS32), (void*)&Analogue_Input_Interrupt_Delta_Unsigned[4] },
                       { RW, uint32, sizeof (UNS32), (void*)&Analogue_Input_Interrupt_Delta_Unsigned[5] },
                       { RW, uint32, sizeof (UNS32), (void*)&Analogue_Input_Interrupt_Delta_Unsigned[6] },
                       { RW, uint32, sizeof (UNS32), (void*)&Analogue_Input_Interrupt_Delta_Unsigned[7] }
                     };

/* index 0x6443 :   Mapped variable Analogue Output Error Mode */
                    UNS8 ObjDict_highestSubIndex_obj6443 = 4; /* number of subindex - 1*/
                    subindex ObjDict_Index6443[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6443 },
                       { RW, uint8, sizeof (UNS8), (void*)&Analogue_Output_Error_Mode[0] },
                       { RW, uint8, sizeof (UNS8), (void*)&Analogue_Output_Error_Mode[1] },
                       { RW, uint8, sizeof (UNS8), (void*)&Analogue_Output_Error_Mode[2] },
                       { RW, uint8, sizeof (UNS8), (void*)&Analogue_Output_Error_Mode[3] }
                     };

/* index 0x6444 :   Mapped variable Analogue Output Error Value Integer */
                    UNS8 ObjDict_highestSubIndex_obj6444 = 4; /* number of subindex - 1*/
                    subindex ObjDict_Index6444[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&ObjDict_highestSubIndex_obj6444 },
                       { RW, int32, sizeof (INTEGER32), (void*)&Analogue_Output_Error_Value_Integer[0] },
                       { RW, int32, sizeof (INTEGER32), (void*)&Analogue_Output_Error_Value_Integer[1] },
                       { RW, int32, sizeof (INTEGER32), (void*)&Analogue_Output_Error_Value_Integer[2] },
                       { RW, int32, sizeof (INTEGER32), (void*)&Analogue_Output_Error_Value_Integer[3] }
                     };

/**************************************************************************/
/* Declaration of pointed variables                                       */
/**************************************************************************/

const indextable ObjDict_objdict[] = 
{
  { (subindex*)ObjDict_Index1000,sizeof(ObjDict_Index1000)/sizeof(ObjDict_Index1000[0]), 0x1000},
  { (subindex*)ObjDict_Index1001,sizeof(ObjDict_Index1001)/sizeof(ObjDict_Index1001[0]), 0x1001},
  { (subindex*)ObjDict_Index1002,sizeof(ObjDict_Index1002)/sizeof(ObjDict_Index1002[0]), 0x1002},
  { (subindex*)ObjDict_Index1005,sizeof(ObjDict_Index1005)/sizeof(ObjDict_Index1005[0]), 0x1005},
  { (subindex*)ObjDict_Index1008,sizeof(ObjDict_Index1008)/sizeof(ObjDict_Index1008[0]), 0x1008},
  { (subindex*)ObjDict_Index1009,sizeof(ObjDict_Index1009)/sizeof(ObjDict_Index1009[0]), 0x1009},
  { (subindex*)ObjDict_Index100A,sizeof(ObjDict_Index100A)/sizeof(ObjDict_Index100A[0]), 0x100A},
  { (subindex*)ObjDict_Index100C,sizeof(ObjDict_Index100C)/sizeof(ObjDict_Index100C[0]), 0x100C},
  { (subindex*)ObjDict_Index100D,sizeof(ObjDict_Index100D)/sizeof(ObjDict_Index100D[0]), 0x100D},
  { (subindex*)ObjDict_Index1010,sizeof(ObjDict_Index1010)/sizeof(ObjDict_Index1010[0]), 0x1010},
  { (subindex*)ObjDict_Index1011,sizeof(ObjDict_Index1011)/sizeof(ObjDict_Index1011[0]), 0x1011},
  { (subindex*)ObjDict_Index1016,sizeof(ObjDict_Index1016)/sizeof(ObjDict_Index1016[0]), 0x1016},
  { (subindex*)ObjDict_Index1017,sizeof(ObjDict_Index1017)/sizeof(ObjDict_Index1017[0]), 0x1017},
  { (subindex*)ObjDict_Index1018,sizeof(ObjDict_Index1018)/sizeof(ObjDict_Index1018[0]), 0x1018},
  { (subindex*)ObjDict_Index1020,sizeof(ObjDict_Index1020)/sizeof(ObjDict_Index1020[0]), 0x1020},
  { (subindex*)ObjDict_Index1200,sizeof(ObjDict_Index1200)/sizeof(ObjDict_Index1200[0]), 0x1200},
  { (subindex*)ObjDict_Index1400,sizeof(ObjDict_Index1400)/sizeof(ObjDict_Index1400[0]), 0x1400},
  { (subindex*)ObjDict_Index1401,sizeof(ObjDict_Index1401)/sizeof(ObjDict_Index1401[0]), 0x1401},
  { (subindex*)ObjDict_Index1600,sizeof(ObjDict_Index1600)/sizeof(ObjDict_Index1600[0]), 0x1600},
  { (subindex*)ObjDict_Index1601,sizeof(ObjDict_Index1601)/sizeof(ObjDict_Index1601[0]), 0x1601},
  { (subindex*)ObjDict_Index1800,sizeof(ObjDict_Index1800)/sizeof(ObjDict_Index1800[0]), 0x1800},
  { (subindex*)ObjDict_Index1801,sizeof(ObjDict_Index1801)/sizeof(ObjDict_Index1801[0]), 0x1801},
  { (subindex*)ObjDict_Index1802,sizeof(ObjDict_Index1802)/sizeof(ObjDict_Index1802[0]), 0x1802},
  { (subindex*)ObjDict_Index1A00,sizeof(ObjDict_Index1A00)/sizeof(ObjDict_Index1A00[0]), 0x1A00},
  { (subindex*)ObjDict_Index1A01,sizeof(ObjDict_Index1A01)/sizeof(ObjDict_Index1A01[0]), 0x1A01},
  { (subindex*)ObjDict_Index1A02,sizeof(ObjDict_Index1A02)/sizeof(ObjDict_Index1A02[0]), 0x1A02},
  { (subindex*)ObjDict_Index6000,sizeof(ObjDict_Index6000)/sizeof(ObjDict_Index6000[0]), 0x6000},
  { (subindex*)ObjDict_Index6002,sizeof(ObjDict_Index6002)/sizeof(ObjDict_Index6002[0]), 0x6002},
  { (subindex*)ObjDict_Index6200,sizeof(ObjDict_Index6200)/sizeof(ObjDict_Index6200[0]), 0x6200},
  { (subindex*)ObjDict_Index6202,sizeof(ObjDict_Index6202)/sizeof(ObjDict_Index6202[0]), 0x6202},
  { (subindex*)ObjDict_Index6206,sizeof(ObjDict_Index6206)/sizeof(ObjDict_Index6206[0]), 0x6206},
  { (subindex*)ObjDict_Index6207,sizeof(ObjDict_Index6207)/sizeof(ObjDict_Index6207[0]), 0x6207},
  { (subindex*)ObjDict_Index6401,sizeof(ObjDict_Index6401)/sizeof(ObjDict_Index6401[0]), 0x6401},
  { (subindex*)ObjDict_Index6411,sizeof(ObjDict_Index6411)/sizeof(ObjDict_Index6411[0]), 0x6411},
  { (subindex*)ObjDict_Index6423,sizeof(ObjDict_Index6423)/sizeof(ObjDict_Index6423[0]), 0x6423},
  { (subindex*)ObjDict_Index6426,sizeof(ObjDict_Index6426)/sizeof(ObjDict_Index6426[0]), 0x6426},
  { (subindex*)ObjDict_Index6443,sizeof(ObjDict_Index6443)/sizeof(ObjDict_Index6443[0]), 0x6443},
  { (subindex*)ObjDict_Index6444,sizeof(ObjDict_Index6444)/sizeof(ObjDict_Index6444[0]), 0x6444},
};

const indextable * ObjDict_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks)
{
	int i;
	*callbacks = NULL;
	switch(wIndex){
		case 0x1000: i = 0;break;
		case 0x1001: i = 1;break;
		case 0x1002: i = 2;break;
		case 0x1005: i = 3;*callbacks = ObjDict_Index1005_callbacks; break;
		case 0x1008: i = 4;break;
		case 0x1009: i = 5;break;
		case 0x100A: i = 6;break;
		case 0x100C: i = 7;break;
		case 0x100D: i = 8;break;
		case 0x1010: i = 9;break;
		case 0x1011: i = 10;break;
		case 0x1016: i = 11;break;
		case 0x1017: i = 12;*callbacks = ObjDict_Index1017_callbacks; break;
		case 0x1018: i = 13;break;
		case 0x1020: i = 14;break;
		case 0x1200: i = 15;break;
		case 0x1400: i = 16;break;
		case 0x1401: i = 17;break;
		case 0x1600: i = 18;break;
		case 0x1601: i = 19;break;
		case 0x1800: i = 20;*callbacks = ObjDict_Index1800_callbacks; break;
		case 0x1801: i = 21;*callbacks = ObjDict_Index1801_callbacks; break;
		case 0x1802: i = 22;*callbacks = ObjDict_Index1802_callbacks; break;
		case 0x1A00: i = 23;break;
		case 0x1A01: i = 24;break;
		case 0x1A02: i = 25;break;
		case 0x6000: i = 26;break;
		case 0x6002: i = 27;break;
		case 0x6200: i = 28;break;
		case 0x6202: i = 29;break;
		case 0x6206: i = 30;break;
		case 0x6207: i = 31;break;
		case 0x6401: i = 32;break;
		case 0x6411: i = 33;break;
		case 0x6423: i = 34;break;
		case 0x6426: i = 35;break;
		case 0x6443: i = 36;break;
		case 0x6444: i = 37;break;
		default:
			*errorCode = OD_NO_SUCH_OBJECT;
			return NULL;
	}
	*errorCode = OD_SUCCESSFUL;
	return &ObjDict_objdict[i];
}

/* 
 * To count at which received SYNC a PDO must be sent.
 * Even if no pdoTransmit are defined, at least one entry is computed
 * for compilations issues.
 */
s_PDO_status ObjDict_PDO_status[3] = {s_PDO_status_Initializer,s_PDO_status_Initializer,s_PDO_status_Initializer};

const quick_index ObjDict_firstIndex = {
  15, /* SDO_SVR */
  0, /* SDO_CLT */
  16, /* PDO_RCV */
  18, /* PDO_RCV_MAP */
  20, /* PDO_TRS */
  23 /* PDO_TRS_MAP */
};

const quick_index ObjDict_lastIndex = {
  15, /* SDO_SVR */
  0, /* SDO_CLT */
  17, /* PDO_RCV */
  19, /* PDO_RCV_MAP */
  22, /* PDO_TRS */
  25 /* PDO_TRS_MAP */
};

const UNS16 ObjDict_ObjdictSize = sizeof(ObjDict_objdict)/sizeof(ObjDict_objdict[0]); 

CO_Data ObjDict_Data = CANOPEN_NODE_DATA_INITIALIZER(ObjDict);

