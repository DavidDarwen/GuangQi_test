/** CAN message encoder/decoder: automatically generated - do not edit
  * Generated by dbcc: See https://github.com/howerj/dbcc */
#ifndef CAN_A_H
#define CAN_A_H

/* If the contents of this file have caused breaking changes for you, you could try using
   an older version of the generator. You can specify this on the command line with
   the -n option. */
#define DBCC_GENERATOR_VERSION (3)

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" { 
#endif

#ifndef PREPACK
#define PREPACK
#endif

#ifndef POSTPACK
#define POSTPACK
#endif

#ifndef DBCC_TIME_STAMP
#define DBCC_TIME_STAMP
typedef uint32_t dbcc_time_stamp_t; /* Time stamp for message; you decide on units */
#endif

#ifndef DBCC_STATUS_ENUM
#define DBCC_STATUS_ENUM
typedef enum {
	DBCC_SIG_STAT_UNINITIALIZED_E = 0, /* Message never sent/received */
	DBCC_SIG_STAT_OK_E            = 1, /* Message ok */
	DBCC_SIG_STAT_ERROR_E         = 2, /* Encode/Decode/Timestamp/Any error */
} dbcc_signal_status_e;
#endif

#define CAN_ID_PAS_3 (354) /* 0x162 */
#define CAN_ID_VCU_21_A (374) /* 0x176 */
#define CAN_ID_BCS_2_A (608) /* 0x260 */
#define CAN_ID_BCS_11_A (618) /* 0x26a */
#define CAN_ID_EBB_2_A (643) /* 0x283 */
#define CAN_ID_VCU_9_A (683) /* 0x2ab */
#define CAN_ID_VCU_13_A (689) /* 0x2b1 */
#define CAN_ID_VCU_2_A (864) /* 0x360 */

typedef PREPACK struct {
	uint8_t FAPA_BCMTurnLampReq; /* scaling 1.0, offset 0.0, units none  */
} POSTPACK can_0x162_PAS_3_t;

typedef PREPACK struct {
	uint16_t VCU_EMS_EngSpd; /* scaling 1.0, offset 0.0, units rpm  */
	uint8_t VCU_EMS_AccPedalActPst; /* scaling 0.4, offset 0.0, units %  */
} POSTPACK can_0x176_VCU_21_A_t;

typedef PREPACK struct {
	uint16_t BCS_VehSpd; /* scaling 0.1, offset 0.0, units km/h  */
} POSTPACK can_0x260_BCS_2_A_t;

typedef PREPACK struct {
	uint16_t BCS_ActVehLongAccel; /* scaling 0.0, offset -21.6, units m/s2  */
} POSTPACK can_0x26a_BCS_11_A_t;

typedef PREPACK struct {
	uint8_t EBB_brkPedPst; /* scaling 0.4, offset 0.0, units %  */
} POSTPACK can_0x283_EBB_2_A_t;

typedef PREPACK struct {
	uint8_t VCU_CrntGearLvl; /* scaling 1.0, offset 0.0, units none  */
} POSTPACK can_0x2ab_VCU_9_A_t;

typedef PREPACK struct {
	uint16_t VCU_ActVehWheelTorq; /* scaling 1.0, offset 0.0, units Nm  */
} POSTPACK can_0x2b1_VCU_13_A_t;

typedef PREPACK struct {
	uint8_t VCU_VehRdySt; /* scaling 1.0, offset 0.0, units none  */
} POSTPACK can_0x360_VCU_2_A_t;

typedef PREPACK struct {
	dbcc_time_stamp_t can_0x162_PAS_3_time_stamp_rx;
	dbcc_time_stamp_t can_0x176_VCU_21_A_time_stamp_rx;
	dbcc_time_stamp_t can_0x260_BCS_2_A_time_stamp_rx;
	dbcc_time_stamp_t can_0x26a_BCS_11_A_time_stamp_rx;
	dbcc_time_stamp_t can_0x283_EBB_2_A_time_stamp_rx;
	dbcc_time_stamp_t can_0x2ab_VCU_9_A_time_stamp_rx;
	dbcc_time_stamp_t can_0x2b1_VCU_13_A_time_stamp_rx;
	dbcc_time_stamp_t can_0x360_VCU_2_A_time_stamp_rx;
	unsigned can_0x162_PAS_3_status : 2;
	unsigned can_0x162_PAS_3_tx : 1;
	unsigned can_0x162_PAS_3_rx : 1;
	unsigned can_0x176_VCU_21_A_status : 2;
	unsigned can_0x176_VCU_21_A_tx : 1;
	unsigned can_0x176_VCU_21_A_rx : 1;
	unsigned can_0x260_BCS_2_A_status : 2;
	unsigned can_0x260_BCS_2_A_tx : 1;
	unsigned can_0x260_BCS_2_A_rx : 1;
	unsigned can_0x26a_BCS_11_A_status : 2;
	unsigned can_0x26a_BCS_11_A_tx : 1;
	unsigned can_0x26a_BCS_11_A_rx : 1;
	unsigned can_0x283_EBB_2_A_status : 2;
	unsigned can_0x283_EBB_2_A_tx : 1;
	unsigned can_0x283_EBB_2_A_rx : 1;
	unsigned can_0x2ab_VCU_9_A_status : 2;
	unsigned can_0x2ab_VCU_9_A_tx : 1;
	unsigned can_0x2ab_VCU_9_A_rx : 1;
	unsigned can_0x2b1_VCU_13_A_status : 2;
	unsigned can_0x2b1_VCU_13_A_tx : 1;
	unsigned can_0x2b1_VCU_13_A_rx : 1;
	unsigned can_0x360_VCU_2_A_status : 2;
	unsigned can_0x360_VCU_2_A_tx : 1;
	unsigned can_0x360_VCU_2_A_rx : 1;
	can_0x162_PAS_3_t can_0x162_PAS_3;
	can_0x176_VCU_21_A_t can_0x176_VCU_21_A;
	can_0x260_BCS_2_A_t can_0x260_BCS_2_A;
	can_0x26a_BCS_11_A_t can_0x26a_BCS_11_A;
	can_0x283_EBB_2_A_t can_0x283_EBB_2_A;
	can_0x2ab_VCU_9_A_t can_0x2ab_VCU_9_A;
	can_0x2b1_VCU_13_A_t can_0x2b1_VCU_13_A;
	can_0x360_VCU_2_A_t can_0x360_VCU_2_A;
} POSTPACK can_obj_can_a_h_t;

int unpack_message(can_obj_can_a_h_t *o, const unsigned long id, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp);
int pack_message(can_obj_can_a_h_t *o, const unsigned long id, uint64_t *data);
int print_message(const can_obj_can_a_h_t *o, const unsigned long id, FILE *output);

int decode_can_0x162_FAPA_BCMTurnLampReq(const can_obj_can_a_h_t *o, uint8_t *out);
int encode_can_0x162_FAPA_BCMTurnLampReq(can_obj_can_a_h_t *o, uint8_t in);


int decode_can_0x176_VCU_EMS_EngSpd(const can_obj_can_a_h_t *o, uint16_t *out);
int encode_can_0x176_VCU_EMS_EngSpd(can_obj_can_a_h_t *o, uint16_t in);
int decode_can_0x176_VCU_EMS_AccPedalActPst(const can_obj_can_a_h_t *o, double *out);
int encode_can_0x176_VCU_EMS_AccPedalActPst(can_obj_can_a_h_t *o, double in);


int decode_can_0x260_BCS_VehSpd(const can_obj_can_a_h_t *o, double *out);
int encode_can_0x260_BCS_VehSpd(can_obj_can_a_h_t *o, double in);


int decode_can_0x26a_BCS_ActVehLongAccel(const can_obj_can_a_h_t *o, double *out);
int encode_can_0x26a_BCS_ActVehLongAccel(can_obj_can_a_h_t *o, double in);


int decode_can_0x283_EBB_brkPedPst(const can_obj_can_a_h_t *o, double *out);
int encode_can_0x283_EBB_brkPedPst(can_obj_can_a_h_t *o, double in);


int decode_can_0x2ab_VCU_CrntGearLvl(const can_obj_can_a_h_t *o, uint8_t *out);
int encode_can_0x2ab_VCU_CrntGearLvl(can_obj_can_a_h_t *o, uint8_t in);


int decode_can_0x2b1_VCU_ActVehWheelTorq(const can_obj_can_a_h_t *o, uint16_t *out);
int encode_can_0x2b1_VCU_ActVehWheelTorq(can_obj_can_a_h_t *o, uint16_t in);


int decode_can_0x360_VCU_VehRdySt(const can_obj_can_a_h_t *o, uint8_t *out);
int encode_can_0x360_VCU_VehRdySt(can_obj_can_a_h_t *o, uint8_t in);


#ifdef __cplusplus
} 
#endif

#endif
