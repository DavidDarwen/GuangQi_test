/* Generated by DBCC, see <https://github.com/howerj/dbcc> */
#include "dbc_file/CAN_GQ.h"
#include <inttypes.h>
#include <assert.h>

#define UNUSED(X) ((void)(X))

static inline uint64_t reverse_byte_order(uint64_t x) {
	x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
	x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
	x = (x & 0x00FF00FF00FF00FF) << 8  | (x & 0xFF00FF00FF00FF00) >> 8;
	return x;
}

static inline int print_helper(int r, int print_return_value) {
	return ((r >= 0) && (print_return_value >= 0)) ? r + print_return_value : -1;
}

static int pack_can_0x162_PAS_3(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* FAPA_BCMTurnLampReq: start-bit 28, length 2, endianess motorola, scaling 1, offset 0 */
	x = ((uint8_t)(o->can_0x162_PAS_3.FAPA_BCMTurnLampReq)) & 0x3;
	x <<= 35; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x162_PAS_3_tx = 1;
	return 8;
}

static int unpack_can_0x162_PAS_3(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* FAPA_BCMTurnLampReq: start-bit 28, length 2, endianess motorola, scaling 1, offset 0 */
	x = (m >> 35) & 0x3;
	o->can_0x162_PAS_3.FAPA_BCMTurnLampReq = x;
	o->can_0x162_PAS_3_rx = 1;
	o->can_0x162_PAS_3_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x162_FAPA_BCMTurnLampReq(const can_obj_can_gq_h_t *o, uint8_t *out) {
	assert(o);
	assert(out);
	uint8_t rval = (uint8_t)(o->can_0x162_PAS_3.FAPA_BCMTurnLampReq);
	*out = rval;
	return 0;
}

int encode_can_0x162_FAPA_BCMTurnLampReq(can_obj_can_gq_h_t *o, uint8_t in) {
	assert(o);
	o->can_0x162_PAS_3.FAPA_BCMTurnLampReq = in;
	return 0;
}

int print_can_0x162_PAS_3(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "FAPA_BCMTurnLampReq = (wire: %.0f)\n", (double)(o->can_0x162_PAS_3.FAPA_BCMTurnLampReq)));
	return r;
}

static int pack_can_0x171_DCU_1(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* DCU_RosSpdAct: start-bit 7, length 16, endianess motorola, scaling 1, offset -32766 */
	x = ((uint16_t)(o->can_0x171_DCU_1.DCU_RosSpdAct)) & 0xffff;
	x <<= 48; 
	m |= x;
	/* DCU_IdcAct: start-bit 23, length 11, endianess motorola, scaling 1, offset -1023 */
	x = ((uint16_t)(o->can_0x171_DCU_1.DCU_IdcAct)) & 0x7ff;
	x <<= 37; 
	m |= x;
	/* DCU_TorqAct: start-bit 34, length 11, endianess motorola, scaling 1, offset -1023 */
	x = ((uint16_t)(o->can_0x171_DCU_1.DCU_TorqAct)) & 0x7ff;
	x <<= 16; 
	m |= x;
	/* DCU_UdcAct: start-bit 28, length 10, endianess motorola, scaling 1, offset 0 */
	x = ((uint16_t)(o->can_0x171_DCU_1.DCU_UdcAct)) & 0x3ff;
	x <<= 27; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x171_DCU_1_tx = 1;
	return 8;
}

static int unpack_can_0x171_DCU_1(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* DCU_RosSpdAct: start-bit 7, length 16, endianess motorola, scaling 1, offset -32766 */
	x = (m >> 48) & 0xffff;
	o->can_0x171_DCU_1.DCU_RosSpdAct = x;
	/* DCU_IdcAct: start-bit 23, length 11, endianess motorola, scaling 1, offset -1023 */
	x = (m >> 37) & 0x7ff;
	o->can_0x171_DCU_1.DCU_IdcAct = x;
	/* DCU_TorqAct: start-bit 34, length 11, endianess motorola, scaling 1, offset -1023 */
	x = (m >> 16) & 0x7ff;
	o->can_0x171_DCU_1.DCU_TorqAct = x;
	/* DCU_UdcAct: start-bit 28, length 10, endianess motorola, scaling 1, offset 0 */
	x = (m >> 27) & 0x3ff;
	o->can_0x171_DCU_1.DCU_UdcAct = x;
	o->can_0x171_DCU_1_rx = 1;
	o->can_0x171_DCU_1_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x171_DCU_RosSpdAct(const can_obj_can_gq_h_t *o, double *out) {
	assert(o);
	assert(out);
	double rval = (double)(o->can_0x171_DCU_1.DCU_RosSpdAct);
	rval += -32766;
	if (rval <= 32767) {
		*out = rval;
		return 0;
	} else {
		*out = (double)0;
		return -1;
	}
}

int encode_can_0x171_DCU_RosSpdAct(can_obj_can_gq_h_t *o, double in) {
	assert(o);
	o->can_0x171_DCU_1.DCU_RosSpdAct = 0;
	if (in > 32767)
		return -1;
	in += 32766;
	o->can_0x171_DCU_1.DCU_RosSpdAct = in;
	return 0;
}

int decode_can_0x171_DCU_IdcAct(const can_obj_can_gq_h_t *o, double *out) {
	assert(o);
	assert(out);
	double rval = (double)(o->can_0x171_DCU_1.DCU_IdcAct);
	rval += -1023;
	if (rval <= 1024) {
		*out = rval;
		return 0;
	} else {
		*out = (double)0;
		return -1;
	}
}

int encode_can_0x171_DCU_IdcAct(can_obj_can_gq_h_t *o, double in) {
	assert(o);
	o->can_0x171_DCU_1.DCU_IdcAct = 0;
	if (in > 1024)
		return -1;
	in += 1023;
	o->can_0x171_DCU_1.DCU_IdcAct = in;
	return 0;
}

int decode_can_0x171_DCU_TorqAct(const can_obj_can_gq_h_t *o, double *out) {
	assert(o);
	assert(out);
	double rval = (double)(o->can_0x171_DCU_1.DCU_TorqAct);
	rval += -1023;
	if (rval <= 1024) {
		*out = rval;
		return 0;
	} else {
		*out = (double)0;
		return -1;
	}
}

int encode_can_0x171_DCU_TorqAct(can_obj_can_gq_h_t *o, double in) {
	assert(o);
	o->can_0x171_DCU_1.DCU_TorqAct = 0;
	if (in > 1024)
		return -1;
	in += 1023;
	o->can_0x171_DCU_1.DCU_TorqAct = in;
	return 0;
}

int decode_can_0x171_DCU_UdcAct(const can_obj_can_gq_h_t *o, uint16_t *out) {
	assert(o);
	assert(out);
	uint16_t rval = (uint16_t)(o->can_0x171_DCU_1.DCU_UdcAct);
	*out = rval;
	return 0;
}

int encode_can_0x171_DCU_UdcAct(can_obj_can_gq_h_t *o, uint16_t in) {
	assert(o);
	o->can_0x171_DCU_1.DCU_UdcAct = in;
	return 0;
}

int print_can_0x171_DCU_1(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "DCU_RosSpdAct = (wire: %.0f)\n", (double)(o->can_0x171_DCU_1.DCU_RosSpdAct)));
	r = print_helper(r, fprintf(output, "DCU_IdcAct = (wire: %.0f)\n", (double)(o->can_0x171_DCU_1.DCU_IdcAct)));
	r = print_helper(r, fprintf(output, "DCU_TorqAct = (wire: %.0f)\n", (double)(o->can_0x171_DCU_1.DCU_TorqAct)));
	r = print_helper(r, fprintf(output, "DCU_UdcAct = (wire: %.0f)\n", (double)(o->can_0x171_DCU_1.DCU_UdcAct)));
	return r;
}

static int pack_can_0x176_VCU_21_A(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* VCU_EMS_EngSpd: start-bit 31, length 16, endianess motorola, scaling 1, offset 0 */
	x = ((uint16_t)(o->can_0x176_VCU_21_A.VCU_EMS_EngSpd)) & 0xffff;
	x <<= 24; 
	m |= x;
	/* VCU_EMS_AccPedalActPst: start-bit 47, length 8, endianess motorola, scaling 0.392, offset 0 */
	x = ((uint8_t)(o->can_0x176_VCU_21_A.VCU_EMS_AccPedalActPst)) & 0xff;
	x <<= 16; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x176_VCU_21_A_tx = 1;
	return 8;
}

static int unpack_can_0x176_VCU_21_A(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* VCU_EMS_EngSpd: start-bit 31, length 16, endianess motorola, scaling 1, offset 0 */
	x = (m >> 24) & 0xffff;
	o->can_0x176_VCU_21_A.VCU_EMS_EngSpd = x;
	/* VCU_EMS_AccPedalActPst: start-bit 47, length 8, endianess motorola, scaling 0.392, offset 0 */
	x = (m >> 16) & 0xff;
	o->can_0x176_VCU_21_A.VCU_EMS_AccPedalActPst = x;
	o->can_0x176_VCU_21_A_rx = 1;
	o->can_0x176_VCU_21_A_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x176_VCU_EMS_EngSpd(const can_obj_can_gq_h_t *o, uint16_t *out) {
	assert(o);
	assert(out);
	uint16_t rval = (uint16_t)(o->can_0x176_VCU_21_A.VCU_EMS_EngSpd);
	*out = rval;
	return 0;
}

int encode_can_0x176_VCU_EMS_EngSpd(can_obj_can_gq_h_t *o, uint16_t in) {
	assert(o);
	o->can_0x176_VCU_21_A.VCU_EMS_EngSpd = in;
	return 0;
}

int decode_can_0x176_VCU_EMS_AccPedalActPst(const can_obj_can_gq_h_t *o, double *out) {
	assert(o);
	assert(out);
	double rval = (double)(o->can_0x176_VCU_21_A.VCU_EMS_AccPedalActPst);
	rval *= 0.392;
	if (rval <= 99.96) {
		*out = rval;
		return 0;
	} else {
		*out = (double)0;
		return -1;
	}
}

int encode_can_0x176_VCU_EMS_AccPedalActPst(can_obj_can_gq_h_t *o, double in) {
	assert(o);
	o->can_0x176_VCU_21_A.VCU_EMS_AccPedalActPst = 0;
	if (in > 99.96)
		return -1;
	in *= 2.55102;
	o->can_0x176_VCU_21_A.VCU_EMS_AccPedalActPst = in;
	return 0;
}

int print_can_0x176_VCU_21_A(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "VCU_EMS_EngSpd = (wire: %.0f)\n", (double)(o->can_0x176_VCU_21_A.VCU_EMS_EngSpd)));
	r = print_helper(r, fprintf(output, "VCU_EMS_AccPedalActPst = (wire: %.0f)\n", (double)(o->can_0x176_VCU_21_A.VCU_EMS_AccPedalActPst)));
	return r;
}

static int pack_can_0x260_BCS_2_A(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* BCS_VehSpd: start-bit 36, length 13, endianess motorola, scaling 0.05625, offset 0 */
	x = ((uint16_t)(o->can_0x260_BCS_2_A.BCS_VehSpd)) & 0x1fff;
	x <<= 16; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x260_BCS_2_A_tx = 1;
	return 6;
}

static int unpack_can_0x260_BCS_2_A(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 6)
		return -1;
	/* BCS_VehSpd: start-bit 36, length 13, endianess motorola, scaling 0.05625, offset 0 */
	x = (m >> 16) & 0x1fff;
	o->can_0x260_BCS_2_A.BCS_VehSpd = x;
	o->can_0x260_BCS_2_A_rx = 1;
	o->can_0x260_BCS_2_A_time_stamp_rx = time_stamp;
	return 6;
}

int decode_can_0x260_BCS_VehSpd(const can_obj_can_gq_h_t *o, double *out) {
	assert(o);
	assert(out);
	double rval = (double)(o->can_0x260_BCS_2_A.BCS_VehSpd);
	rval *= 0.05625;
	if (rval <= 240) {
		*out = rval;
		return 0;
	} else {
		*out = (double)0;
		return -1;
	}
}

int encode_can_0x260_BCS_VehSpd(can_obj_can_gq_h_t *o, double in) {
	assert(o);
	o->can_0x260_BCS_2_A.BCS_VehSpd = 0;
	if (in > 240)
		return -1;
	in *= 17.7778;
	o->can_0x260_BCS_2_A.BCS_VehSpd = in;
	return 0;
}

int print_can_0x260_BCS_2_A(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "BCS_VehSpd = (wire: %.0f)\n", (double)(o->can_0x260_BCS_2_A.BCS_VehSpd)));
	return r;
}

static int pack_can_0x26a_BCS_11_A(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* BCS_ActVehLongAccel: start-bit 7, length 12, endianess motorola, scaling 0.0271267, offset -21.593 */
	x = ((uint16_t)(o->can_0x26a_BCS_11_A.BCS_ActVehLongAccel)) & 0xfff;
	x <<= 52; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x26a_BCS_11_A_tx = 1;
	return 8;
}

static int unpack_can_0x26a_BCS_11_A(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* BCS_ActVehLongAccel: start-bit 7, length 12, endianess motorola, scaling 0.0271267, offset -21.593 */
	x = (m >> 52) & 0xfff;
	o->can_0x26a_BCS_11_A.BCS_ActVehLongAccel = x;
	o->can_0x26a_BCS_11_A_rx = 1;
	o->can_0x26a_BCS_11_A_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x26a_BCS_ActVehLongAccel(const can_obj_can_gq_h_t *o, double *out) {
	assert(o);
	assert(out);
	double rval = (double)(o->can_0x26a_BCS_11_A.BCS_ActVehLongAccel);
	rval *= 0.0271267;
	rval += -21.593;
	if (rval <= 21.593) {
		*out = rval;
		return 0;
	} else {
		*out = (double)0;
		return -1;
	}
}

int encode_can_0x26a_BCS_ActVehLongAccel(can_obj_can_gq_h_t *o, double in) {
	assert(o);
	o->can_0x26a_BCS_11_A.BCS_ActVehLongAccel = 0;
	if (in > 21.593)
		return -1;
	in += 21.593;
	in *= 36.864;
	o->can_0x26a_BCS_11_A.BCS_ActVehLongAccel = in;
	return 0;
}

int print_can_0x26a_BCS_11_A(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "BCS_ActVehLongAccel = (wire: %.0f)\n", (double)(o->can_0x26a_BCS_11_A.BCS_ActVehLongAccel)));
	return r;
}

static int pack_can_0x283_EBB_2_A(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* EBB_brkPedPst: start-bit 39, length 8, endianess motorola, scaling 0.392, offset 0 */
	x = ((uint8_t)(o->can_0x283_EBB_2_A.EBB_brkPedPst)) & 0xff;
	x <<= 24; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x283_EBB_2_A_tx = 1;
	return 8;
}

static int unpack_can_0x283_EBB_2_A(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* EBB_brkPedPst: start-bit 39, length 8, endianess motorola, scaling 0.392, offset 0 */
	x = (m >> 24) & 0xff;
	o->can_0x283_EBB_2_A.EBB_brkPedPst = x;
	o->can_0x283_EBB_2_A_rx = 1;
	o->can_0x283_EBB_2_A_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x283_EBB_brkPedPst(const can_obj_can_gq_h_t *o, double *out) {
	assert(o);
	assert(out);
	double rval = (double)(o->can_0x283_EBB_2_A.EBB_brkPedPst);
	rval *= 0.392;
	if (rval <= 99.96) {
		*out = rval;
		return 0;
	} else {
		*out = (double)0;
		return -1;
	}
}

int encode_can_0x283_EBB_brkPedPst(can_obj_can_gq_h_t *o, double in) {
	assert(o);
	o->can_0x283_EBB_2_A.EBB_brkPedPst = 0;
	if (in > 99.96)
		return -1;
	in *= 2.55102;
	o->can_0x283_EBB_2_A.EBB_brkPedPst = in;
	return 0;
}

int print_can_0x283_EBB_2_A(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "EBB_brkPedPst = (wire: %.0f)\n", (double)(o->can_0x283_EBB_2_A.EBB_brkPedPst)));
	return r;
}

static int pack_can_0x2ab_VCU_9_A(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* VCU_CrntGearLvl: start-bit 38, length 3, endianess motorola, scaling 1, offset 0 */
	x = ((uint8_t)(o->can_0x2ab_VCU_9_A.VCU_CrntGearLvl)) & 0x7;
	x <<= 28; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x2ab_VCU_9_A_tx = 1;
	return 8;
}

static int unpack_can_0x2ab_VCU_9_A(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* VCU_CrntGearLvl: start-bit 38, length 3, endianess motorola, scaling 1, offset 0 */
	x = (m >> 28) & 0x7;
	o->can_0x2ab_VCU_9_A.VCU_CrntGearLvl = x;
	o->can_0x2ab_VCU_9_A_rx = 1;
	o->can_0x2ab_VCU_9_A_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x2ab_VCU_CrntGearLvl(const can_obj_can_gq_h_t *o, uint8_t *out) {
	assert(o);
	assert(out);
	uint8_t rval = (uint8_t)(o->can_0x2ab_VCU_9_A.VCU_CrntGearLvl);
	if (rval <= 0) {
		*out = rval;
		return 0;
	} else {
		*out = (uint8_t)0;
		return -1;
	}
}

int encode_can_0x2ab_VCU_CrntGearLvl(can_obj_can_gq_h_t *o, uint8_t in) {
	assert(o);
	o->can_0x2ab_VCU_9_A.VCU_CrntGearLvl = 0;
	if (in > 0)
		return -1;
	o->can_0x2ab_VCU_9_A.VCU_CrntGearLvl = in;
	return 0;
}

int print_can_0x2ab_VCU_9_A(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "VCU_CrntGearLvl = (wire: %.0f)\n", (double)(o->can_0x2ab_VCU_9_A.VCU_CrntGearLvl)));
	return r;
}

static int pack_can_0x2ad_BMS_18(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* BMS_BattSocDisp: start-bit 29, length 10, endianess motorola, scaling 0.1, offset 0 */
	x = ((uint16_t)(o->can_0x2ad_BMS_18.BMS_BattSocDisp)) & 0x3ff;
	x <<= 28; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x2ad_BMS_18_tx = 1;
	return 8;
}

static int unpack_can_0x2ad_BMS_18(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* BMS_BattSocDisp: start-bit 29, length 10, endianess motorola, scaling 0.1, offset 0 */
	x = (m >> 28) & 0x3ff;
	o->can_0x2ad_BMS_18.BMS_BattSocDisp = x;
	o->can_0x2ad_BMS_18_rx = 1;
	o->can_0x2ad_BMS_18_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x2ad_BMS_BattSocDisp(const can_obj_can_gq_h_t *o, double *out) {
	assert(o);
	assert(out);
	double rval = (double)(o->can_0x2ad_BMS_18.BMS_BattSocDisp);
	rval *= 0.1;
	if (rval <= 100) {
		*out = rval;
		return 0;
	} else {
		*out = (double)0;
		return -1;
	}
}

int encode_can_0x2ad_BMS_BattSocDisp(can_obj_can_gq_h_t *o, double in) {
	assert(o);
	o->can_0x2ad_BMS_18.BMS_BattSocDisp = 0;
	if (in > 100)
		return -1;
	in *= 10;
	o->can_0x2ad_BMS_18.BMS_BattSocDisp = in;
	return 0;
}

int print_can_0x2ad_BMS_18(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "BMS_BattSocDisp = (wire: %.0f)\n", (double)(o->can_0x2ad_BMS_18.BMS_BattSocDisp)));
	return r;
}

static int pack_can_0x2b1_VCU_13_A(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* VCU_ActVehWheelTorq: start-bit 39, length 14, endianess motorola, scaling 1, offset 0 */
	x = ((uint16_t)(o->can_0x2b1_VCU_13_A.VCU_ActVehWheelTorq)) & 0x3fff;
	x <<= 18; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x2b1_VCU_13_A_tx = 1;
	return 8;
}

static int unpack_can_0x2b1_VCU_13_A(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* VCU_ActVehWheelTorq: start-bit 39, length 14, endianess motorola, scaling 1, offset 0 */
	x = (m >> 18) & 0x3fff;
	o->can_0x2b1_VCU_13_A.VCU_ActVehWheelTorq = x;
	o->can_0x2b1_VCU_13_A_rx = 1;
	o->can_0x2b1_VCU_13_A_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x2b1_VCU_ActVehWheelTorq(const can_obj_can_gq_h_t *o, uint16_t *out) {
	assert(o);
	assert(out);
	uint16_t rval = (uint16_t)(o->can_0x2b1_VCU_13_A.VCU_ActVehWheelTorq);
	if (rval <= 10000) {
		*out = rval;
		return 0;
	} else {
		*out = (uint16_t)0;
		return -1;
	}
}

int encode_can_0x2b1_VCU_ActVehWheelTorq(can_obj_can_gq_h_t *o, uint16_t in) {
	assert(o);
	o->can_0x2b1_VCU_13_A.VCU_ActVehWheelTorq = 0;
	if (in > 10000)
		return -1;
	o->can_0x2b1_VCU_13_A.VCU_ActVehWheelTorq = in;
	return 0;
}

int print_can_0x2b1_VCU_13_A(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "VCU_ActVehWheelTorq = (wire: %.0f)\n", (double)(o->can_0x2b1_VCU_13_A.VCU_ActVehWheelTorq)));
	return r;
}

static int pack_can_0x360_VCU_2_A(can_obj_can_gq_h_t *o, uint64_t *data) {
	assert(o);
	assert(data);
	register uint64_t x;
	register uint64_t m = 0;
	/* VCU_VehRdySt: start-bit 22, length 1, endianess motorola, scaling 1, offset 0 */
	x = ((uint8_t)(o->can_0x360_VCU_2_A.VCU_VehRdySt)) & 0x1;
	x <<= 46; 
	m |= x;
	*data = reverse_byte_order(m);
	o->can_0x360_VCU_2_A_tx = 1;
	return 8;
}

static int unpack_can_0x360_VCU_2_A(can_obj_can_gq_h_t *o, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(dlc <= 8);
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if (dlc < 8)
		return -1;
	/* VCU_VehRdySt: start-bit 22, length 1, endianess motorola, scaling 1, offset 0 */
	x = (m >> 46) & 0x1;
	o->can_0x360_VCU_2_A.VCU_VehRdySt = x;
	o->can_0x360_VCU_2_A_rx = 1;
	o->can_0x360_VCU_2_A_time_stamp_rx = time_stamp;
	return 8;
}

int decode_can_0x360_VCU_VehRdySt(const can_obj_can_gq_h_t *o, uint8_t *out) {
	assert(o);
	assert(out);
	uint8_t rval = (uint8_t)(o->can_0x360_VCU_2_A.VCU_VehRdySt);
	*out = rval;
	return 0;
}

int encode_can_0x360_VCU_VehRdySt(can_obj_can_gq_h_t *o, uint8_t in) {
	assert(o);
	o->can_0x360_VCU_2_A.VCU_VehRdySt = in;
	return 0;
}

int print_can_0x360_VCU_2_A(const can_obj_can_gq_h_t *o, FILE *output) {
	assert(o);
	assert(output);
	int r = 0;
	r = print_helper(r, fprintf(output, "VCU_VehRdySt = (wire: %.0f)\n", (double)(o->can_0x360_VCU_2_A.VCU_VehRdySt)));
	return r;
}

int unpack_message(can_obj_can_gq_h_t *o, const unsigned long id, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp) {
	assert(o);
	assert(id < (1ul << 29)); /* 29-bit CAN ID is largest possible */
	assert(dlc <= 8);         /* Maximum of 8 bytes in a CAN packet */
	switch (id) {
	case 0x162: return unpack_can_0x162_PAS_3(o, data, dlc, time_stamp);
	case 0x171: return unpack_can_0x171_DCU_1(o, data, dlc, time_stamp);
	case 0x176: return unpack_can_0x176_VCU_21_A(o, data, dlc, time_stamp);
	case 0x260: return unpack_can_0x260_BCS_2_A(o, data, dlc, time_stamp);
	case 0x26a: return unpack_can_0x26a_BCS_11_A(o, data, dlc, time_stamp);
	case 0x283: return unpack_can_0x283_EBB_2_A(o, data, dlc, time_stamp);
	case 0x2ab: return unpack_can_0x2ab_VCU_9_A(o, data, dlc, time_stamp);
	case 0x2ad: return unpack_can_0x2ad_BMS_18(o, data, dlc, time_stamp);
	case 0x2b1: return unpack_can_0x2b1_VCU_13_A(o, data, dlc, time_stamp);
	case 0x360: return unpack_can_0x360_VCU_2_A(o, data, dlc, time_stamp);
	default: break; 
	}
	return -1; 
}

int pack_message(can_obj_can_gq_h_t *o, const unsigned long id, uint64_t *data) {
	assert(o);
	assert(id < (1ul << 29)); /* 29-bit CAN ID is largest possible */
	switch (id) {
	case 0x162: return pack_can_0x162_PAS_3(o, data);
	case 0x171: return pack_can_0x171_DCU_1(o, data);
	case 0x176: return pack_can_0x176_VCU_21_A(o, data);
	case 0x260: return pack_can_0x260_BCS_2_A(o, data);
	case 0x26a: return pack_can_0x26a_BCS_11_A(o, data);
	case 0x283: return pack_can_0x283_EBB_2_A(o, data);
	case 0x2ab: return pack_can_0x2ab_VCU_9_A(o, data);
	case 0x2ad: return pack_can_0x2ad_BMS_18(o, data);
	case 0x2b1: return pack_can_0x2b1_VCU_13_A(o, data);
	case 0x360: return pack_can_0x360_VCU_2_A(o, data);
	default: break; 
	}
	return -1; 
}

int print_message(const can_obj_can_gq_h_t *o, const unsigned long id, FILE *output) {
	assert(o);
	assert(id < (1ul << 29)); /* 29-bit CAN ID is largest possible */
	assert(output);
	switch (id) {
	case 0x162: return print_can_0x162_PAS_3(o, output);
	case 0x171: return print_can_0x171_DCU_1(o, output);
	case 0x176: return print_can_0x176_VCU_21_A(o, output);
	case 0x260: return print_can_0x260_BCS_2_A(o, output);
	case 0x26a: return print_can_0x26a_BCS_11_A(o, output);
	case 0x283: return print_can_0x283_EBB_2_A(o, output);
	case 0x2ab: return print_can_0x2ab_VCU_9_A(o, output);
	case 0x2ad: return print_can_0x2ad_BMS_18(o, output);
	case 0x2b1: return print_can_0x2b1_VCU_13_A(o, output);
	case 0x360: return print_can_0x360_VCU_2_A(o, output);
	default: break; 
	}
	return -1; 
}

