#ifndef _LCEC_MECA500_H_
#define _LCEC_MECA500_H_

#include "lcec.h"

#define LCEC_MECA500_VID 0x00ecade1
#define LCEC_MECA500_PID 0x0004d500

#define LCEC_MECA500_PDOS 3

int lcec_meca500_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs);

#endif
